#include "stdafx.h"
#include "mcp_server.h"
#include "mcp_common.h"

#include <winsock2.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "../../xrEngine/xr_ioconsole.h"
#include "../../xrEngine/xr_ioc_cmd.h"
#include "../../xrEngine/IGame_Level.h"
#include "../../xrEngine/Render.h"
#include "../MainMenu.h"

#pragma comment(lib, "ws2_32.lib")

namespace mcp
{
namespace
{
constexpr u16 default_port = 47100;
constexpr long long default_timeout_ms = 10000;
constexpr long long max_timeout_ms = 600000;
constexpr size_t max_header_bytes = 64 * 1024;
constexpr size_t max_body_bytes = 16 * 1024 * 1024;
constexpr DWORD recv_timeout_ms = 5000;
constexpr ULONGLONG stall_ms = 2000;
constexpr long long max_log_lines = 5000;
constexpr size_t log_scan_chunk = 4096;

const char instructions[] =
	"Live access to a running X-Ray engine (S.T.A.L.K.E.R. Anomaly). "
	"lua runs code in the game's Lua state, console runs engine console commands, screenshot captures the screen. "
	"status and log_tail answer even while the game is loading or its main thread is stuck. "
	"Tools registered by game scripts (mcp.register_tool) are listed after the built-in ones.";

enum class Thread
{
	net,
	main
};

struct Builtin
{
	const char* name;
	Thread thread;
	const char* description;
	const char* schema;
};

const Builtin builtins[] = {
	{
		"status", Thread::net,
		"Engine state: level name or main menu, whether the main menu is open, paused, frame counter, and whether the main thread is ticking.",
		R"({"type":"object","properties":{}})"
	},
	{
		"log_tail", Thread::net,
		"Last lines of the engine log, optionally only the lines containing a substring.",
		R"({"type":"object","properties":{"lines":{"type":"integer","description":"How many lines to return (default 100)."},"filter":{"type":"string","description":"Case-insensitive substring; only matching lines are returned."}}})"
	},
	{
		"lua", Thread::main,
		"Run Lua (LuaJIT 5.1) in the game's script state on the main thread at the next frame; works in the main menu and while paused. "
		"A single expression is returned as the result; a block needs an explicit return. "
		"Variables you assign persist between calls in a private environment that reads through to the game globals; write a real global with _G.name = value. "
		"Returns JSON {result, output (print), log (engine log lines written during the call), error, ms}. "
		"Game objects are returned as {id, name, section}, vectors as {x, y, z}.",
		R"({"type":"object","properties":{"code":{"type":"string"},"timeout_ms":{"type":"integer","description":"How long to wait for the main thread (default 10000)."}},"required":["code"]})"
	},
	{
		"console", Thread::main,
		"Execute an engine console command. Returns the log lines it produced.",
		R"({"type":"object","properties":{"command":{"type":"string"}},"required":["command"]})"
	},
	{
		"cvar_get", Thread::main,
		"Current value and allowed values of one console command.",
		R"({"type":"object","properties":{"name":{"type":"string"}},"required":["name"]})"
	},
	{
		"cvar_list", Thread::main,
		"Console commands with their current values. Use prefix to narrow the list.",
		R"({"type":"object","properties":{"prefix":{"type":"string"}}})"
	},
	{
		"screenshot", Thread::main,
		"Capture the current frame at full game resolution, with HUD and UI. Format follows r_screenshot_mode (jpg, otherwise png).",
		R"({"type":"object","properties":{}})"
	},
};

struct Job
{
	std::string tool;
	Json args;
	ToolResult result;
	bool done = false;
	bool abandoned = false;
};

struct Status
{
	u32 frame = 0;
	ULONGLONG tick = 0;
	bool in_level = false;
	std::string level;
	bool main_menu = false;
	bool paused = false;
};

std::mutex g_mutex;
std::condition_variable g_cv;
std::deque<std::shared_ptr<Job>> g_queue;
Status g_status;
bool g_stopping = false;

SOCKET g_listen = INVALID_SOCKET;
std::thread g_thread;
bool g_running = false;

const Builtin* find_builtin(const std::string& name)
{
	for (const Builtin& b : builtins)
		if (name == b.name)
			return &b;
	return nullptr;
}

std::string lower(std::string s)
{
	std::transform(s.begin(), s.end(), s.begin(), [](char c) { return static_cast<char>(tolower(static_cast<u8>(c))); });
	return s;
}

bool has_param(const char* params, const char* flag)
{
	size_t len = xr_strlen(flag);
	for (const char* p = strstr(params, flag); p; p = strstr(p + 1, flag))
	{
		bool starts = p == params || p[-1] == ' ';
		bool ends = p[len] == 0 || p[len] == ' ';
		if (starts && ends)
			return true;
	}
	return false;
}

u16 port_from_params()
{
	const char* p = strstr(Core.Params, "-mcp_port ");
	int port = 0;
	if (p && sscanf(p + xr_strlen("-mcp_port "), "%d", &port) == 1 && port > 0 && port < 65536)
		return static_cast<u16>(port);
	return default_port;
}

// ---------------------------------------------------------------- tools

ToolResult run_status()
{
	Status s;
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		s = g_status;
	}
	ULONGLONG since = s.tick ? GetTickCount64() - s.tick : 0;
	const char* main_thread = !s.tick ? "no frame yet" : since > stall_ms ? "not ticking (loading screen or stuck)" : "running";

	ToolResult r;
	r.text(dump(Json::object{
		{"main_thread", main_thread},
		{"ms_since_last_frame", static_cast<double>(since)},
		{"frame", static_cast<double>(s.frame)},
		{"level", s.in_level ? Json(s.level) : Json()},
		{"main_menu_open", s.main_menu},
		{"paused", s.paused},
	}));
	return r;
}

bool contains_ci(const xr_string& line, const std::string& lowered_needle)
{
	auto it = std::search(line.begin(), line.end(), lowered_needle.begin(), lowered_needle.end(),
		[](char a, char b) { return static_cast<char>(tolower(static_cast<u8>(a))) == b; });
	return it != line.end();
}

ToolResult run_log_tail(const Json& args)
{
	size_t want = static_cast<size_t>(std::clamp(get_int(args, "lines", 100), 1LL, max_log_lines));
	std::string filter = lower(to_game(get_str(args, "filter")));

	std::vector<std::string> picked;
	size_t end = LogLineCount();
	while (end > 0 && picked.size() < want)
	{
		size_t begin = end > log_scan_chunk ? end - log_scan_chunk : 0;
		xr_vector<xr_string> lines;
		LogCopyLines(begin, end, lines);
		for (size_t i = lines.size(); i-- > 0 && picked.size() < want;)
			if (filter.empty() || contains_ci(lines[i], filter))
				picked.push_back(to_utf8(lines[i].c_str(), lines[i].size()));
		end = begin;
	}

	std::string text;
	for (auto it = picked.rbegin(); it != picked.rend(); ++it)
	{
		text += *it;
		text += '\n';
	}

	ToolResult r;
	r.text(text.empty() ? "(no matching lines)" : text);
	return r;
}

ToolResult run_console(const std::string& command)
{
	ToolResult r;
	if (command.empty())
	{
		r.fail("command is empty");
		return r;
	}

	size_t mark = log_mark();
	{
		CConsole::ScriptCallerScope scope("mcp");
		Console->Execute(to_game(command).c_str());
	}
	r.text(dump(Json::object{{"log", log_since(mark)}}));
	return r;
}

ToolResult run_cvar_get(const std::string& name)
{
	ToolResult r;
	IConsole_Command* command = Console->GetCommand(to_game(name).c_str());
	if (!command)
	{
		r.fail("unknown console command: " + name);
		return r;
	}

	IConsole_Command::TStatus status;
	status[0] = 0;
	command->Status(status);
	IConsole_Command::TInfo info;
	info[0] = 0;
	command->Info(info);

	r.text(dump(Json::object{
		{"name", name},
		{"value", to_utf8(status)},
		{"info", to_utf8(info)},
	}));
	return r;
}

ToolResult run_cvar_list(const std::string& prefix)
{
	std::string game_prefix = to_game(prefix);
	Json::array list;
	for (const auto& entry : Console->Commands)
	{
		if (!game_prefix.empty() && strncmp(entry.first, game_prefix.c_str(), game_prefix.size()) != 0)
			continue;
		IConsole_Command::TStatus status;
		status[0] = 0;
		entry.second->Status(status);
		list.push_back(Json::object{{"name", to_utf8(entry.first)}, {"value", to_utf8(status)}});
	}

	ToolResult r;
	r.text(dump(list));
	return r;
}

ToolResult run_screenshot()
{
	ToolResult r;
	CMemoryWriter image;
	::Render->Screenshot(IRender_interface::SM_NORMAL, image);

	const u8* data = image.pointer();
	size_t size = image.size();
	const char* mime = nullptr;
	if (size >= 3 && data[0] == 0xFF && data[1] == 0xD8 && data[2] == 0xFF)
		mime = "image/jpeg";
	else if (size >= 8 && memcmp(data, "\x89PNG\r\n\x1a\n", 8) == 0)
		mime = "image/png";

	if (!mime)
	{
		r.fail("the renderer returned no image (screenshots are unavailable while HDR10 output is on)");
		return r;
	}

	r.content.push_back(Json::object{{"type", "image"}, {"data", base64(data, size)}, {"mimeType", mime}});
	r.text(std::to_string(Device.dwWidth) + "x" + std::to_string(Device.dwHeight));
	return r;
}

ToolResult run_on_main_thread(const std::string& tool, const Json& args)
{
	if (tool == "lua")
		return lua_run(get_str(args, "code"));
	if (tool == "console")
		return run_console(get_str(args, "command"));
	if (tool == "cvar_get")
		return run_cvar_get(get_str(args, "name"));
	if (tool == "cvar_list")
		return run_cvar_list(get_str(args, "prefix"));
	if (tool == "screenshot")
		return run_screenshot();
	return lua_call_tool(tool, args);
}

ToolResult queue_for_main_thread(const std::string& tool, const Json& args)
{
	long long timeout = std::clamp(get_int(args, "timeout_ms", default_timeout_ms), 1LL, max_timeout_ms);

	auto job = std::make_shared<Job>();
	job->tool = tool;
	job->args = args;

	std::unique_lock<std::mutex> lock(g_mutex);
	g_queue.push_back(job);
	g_cv.wait_for(lock, std::chrono::milliseconds(timeout), [&] { return job->done || g_stopping; });
	if (job->done)
		return job->result;

	job->abandoned = true;
	ToolResult r;
	r.fail(g_stopping
		? "the game is shutting down"
		: "timed out after " + std::to_string(timeout) + " ms waiting for the main thread (loading screen, or the game is stuck: see status)");
	return r;
}

class Pump : public pureFrame
{
public:
	void _BCL OnFrame() override
	{
		update_status();

		std::deque<std::shared_ptr<Job>> jobs;
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			jobs.swap(g_queue);
		}

		for (const auto& job : jobs)
		{
			{
				std::lock_guard<std::mutex> lock(g_mutex);
				if (job->abandoned)
					continue;
			}
			ToolResult result = run_on_main_thread(job->tool, job->args);
			{
				std::lock_guard<std::mutex> lock(g_mutex);
				job->result = std::move(result);
				job->done = true;
			}
			g_cv.notify_all();
		}
	}

private:
	static void update_status()
	{
		Status s;
		s.frame = Device.dwFrame;
		s.tick = GetTickCount64();
		s.in_level = g_pGameLevel != nullptr;
		if (s.in_level)
			s.level = to_utf8(g_pGameLevel->name().c_str());
		s.main_menu = MainMenu() && MainMenu()->IsActive();
		s.paused = Device.Paused();

		std::lock_guard<std::mutex> lock(g_mutex);
		g_status = std::move(s);
	}
};

Pump g_pump;

// ---------------------------------------------------------------- json-rpc

Json::array tool_list()
{
	Json::array tools;
	for (const Builtin& b : builtins)
	{
		std::string err;
		tools.push_back(Json::object{{"name", b.name}, {"description", b.description}, {"inputSchema", Json::parse(b.schema, err)}});
	}
	for (Json& tool : script_tool_list())
		tools.push_back(std::move(tool));
	return tools;
}

Json rpc_result(const Json& id, Json result)
{
	return Json::object{{"jsonrpc", "2.0"}, {"id", id}, {"result", std::move(result)}};
}

Json rpc_error(const Json& id, int code, const std::string& message)
{
	return Json::object{{"jsonrpc", "2.0"}, {"id", id}, {"error", Json::object{{"code", code}, {"message", message}}}};
}

Json call_tool(const Json& id, const Json& params)
{
	std::string name = get_str(params, "name");
	const Json& arguments = params["arguments"];
	Json args = arguments.is_object() ? arguments : Json(Json::object());

	const Builtin* builtin = find_builtin(name);
	if (!builtin && !script_tool_exists(name))
		return rpc_error(id, -32602, "unknown tool: " + name);

	auto started = std::chrono::steady_clock::now();
	ToolResult r;
	if (builtin && builtin->thread == Thread::net)
		r = name == "status" ? run_status() : run_log_tail(args);
	else
		r = queue_for_main_thread(name, args);

	long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started).count();
	Msg("* [mcp] %s: %s, %lld ms", name.c_str(), r.is_error ? "error" : "ok", ms);
	return rpc_result(id, Json::object{{"content", std::move(r.content)}, {"isError", r.is_error}});
}

Json dispatch(const Json& id, const std::string& method, const Json& params)
{
	if (method == "initialize")
	{
		// every feature used here exists in all published protocol versions, so accept the client's
		return rpc_result(id, Json::object{
			{"protocolVersion", get_str(params, "protocolVersion", "2025-06-18")},
			{"capabilities", Json::object{{"tools", Json::object{{"listChanged", false}}}}},
			{"serverInfo", Json::object{{"name", "xray-monolith"}, {"version", "1.0"}}},
			{"instructions", instructions},
		});
	}
	if (method == "ping")
		return rpc_result(id, Json::object());
	if (method == "tools/list")
		return rpc_result(id, Json::object{{"tools", tool_list()}});
	if (method == "tools/call")
		return call_tool(id, params);
	return rpc_error(id, -32601, "method not found: " + method);
}

// ---------------------------------------------------------------- http

struct Request
{
	std::string method;
	std::string path;
	std::unordered_map<std::string, std::string> headers;
	std::string body;
};

enum class ReadResult
{
	ok,
	closed,
	bad,
	too_large,
	length_required
};

std::string header(const Request& req, const char* name)
{
	auto it = req.headers.find(name);
	return it == req.headers.end() ? std::string() : it->second;
}

ReadResult read_request(SOCKET s, Request& req)
{
	std::string buf;
	char chunk[8192];
	size_t header_end = std::string::npos;
	while (header_end == std::string::npos)
	{
		if (buf.size() > max_header_bytes)
			return ReadResult::too_large;
		int n = recv(s, chunk, sizeof(chunk), 0);
		if (n <= 0)
			return ReadResult::closed;
		buf.append(chunk, n);
		header_end = buf.find("\r\n\r\n");
	}

	size_t line_end = buf.find("\r\n");
	std::string request_line = buf.substr(0, line_end);
	size_t sp1 = request_line.find(' ');
	size_t sp2 = sp1 == std::string::npos ? std::string::npos : request_line.find(' ', sp1 + 1);
	if (sp2 == std::string::npos)
		return ReadResult::bad;
	req.method = request_line.substr(0, sp1);
	req.path = request_line.substr(sp1 + 1, sp2 - sp1 - 1);

	for (size_t pos = line_end + 2; pos < header_end;)
	{
		size_t eol = buf.find("\r\n", pos);
		std::string line = buf.substr(pos, eol - pos);
		size_t colon = line.find(':');
		if (colon != std::string::npos)
		{
			size_t value = line.find_first_not_of(" \t", colon + 1);
			size_t value_end = line.find_last_not_of(" \t");
			req.headers[lower(line.substr(0, colon))] =
				value == std::string::npos ? std::string() : line.substr(value, value_end - value + 1);
		}
		pos = eol + 2;
	}

	if (req.headers.count("transfer-encoding"))
		return ReadResult::length_required;
	size_t length = static_cast<size_t>(_strtoui64(header(req, "content-length").c_str(), nullptr, 10));
	if (length > max_body_bytes)
		return ReadResult::too_large;

	req.body = buf.substr(header_end + 4);
	while (req.body.size() < length)
	{
		int n = recv(s, chunk, sizeof(chunk), 0);
		if (n <= 0)
			return ReadResult::closed;
		req.body.append(chunk, n);
	}
	req.body.resize(length);
	return ReadResult::ok;
}

void send_all(SOCKET s, const std::string& data)
{
	const char* p = data.data();
	size_t left = data.size();
	while (left)
	{
		int sent = send(s, p, static_cast<int>(std::min<size_t>(left, 1 << 20)), 0);
		if (sent <= 0)
			return;
		p += sent;
		left -= sent;
	}
}

void respond(SOCKET s, int code, const char* reason, const std::string& body = std::string(), const char* extra_headers = "")
{
	std::string head = "HTTP/1.1 " + std::to_string(code) + " " + reason + "\r\n";
	if (!body.empty())
		head += "Content-Type: application/json\r\n";
	head += "Content-Length: " + std::to_string(body.size()) + "\r\n";
	head += extra_headers;
	head += "Connection: close\r\n\r\n";
	send_all(s, head + body);
}

bool origin_allowed(const std::string& origin)
{
	if (origin.empty())
		return true;
	for (const char* host : {"http://127.0.0.1", "http://localhost", "http://[::1]"})
	{
		size_t n = xr_strlen(host);
		if (origin.compare(0, n, host) == 0 && (origin.size() == n || origin[n] == ':'))
			return true;
	}
	return false;
}

void serve(SOCKET s)
{
	Request req;
	switch (read_request(s, req))
	{
	case ReadResult::closed:
		return;
	case ReadResult::bad:
		respond(s, 400, "Bad Request");
		return;
	case ReadResult::too_large:
		respond(s, 413, "Payload Too Large");
		return;
	case ReadResult::length_required:
		respond(s, 411, "Length Required");
		return;
	case ReadResult::ok:
		break;
	}

	if (req.path.substr(0, req.path.find('?')) != "/mcp")
	{
		respond(s, 404, "Not Found");
		return;
	}
	if (!origin_allowed(header(req, "origin")))
	{
		respond(s, 403, "Forbidden");
		return;
	}
	if (req.method != "POST")
	{
		respond(s, 405, "Method Not Allowed", std::string(), "Allow: POST\r\n");
		return;
	}

	std::string err;
	Json message = Json::parse(req.body, err);
	if (!err.empty())
	{
		respond(s, 400, "Bad Request", dump(rpc_error(Json(), -32700, "parse error: " + err)));
		return;
	}
	if (!message.is_object())
	{
		respond(s, 400, "Bad Request", dump(rpc_error(Json(), -32600, "expected a single JSON-RPC message")));
		return;
	}

	const Json::object& fields = message.object_items();
	auto id = fields.find("id");
	if (id == fields.end() || !fields.count("method"))
	{
		respond(s, 202, "Accepted");
		return;
	}

	const Json& params = message["params"];
	Json response = dispatch(id->second, get_str(message, "method"), params.is_object() ? params : Json(Json::object()));
	respond(s, 200, "OK", dump(response));
}

void server_loop(SOCKET listener)
{
	thread_name("X-RAY MCP server");
	for (;;)
	{
		SOCKET client = accept(listener, nullptr, nullptr);
		if (client == INVALID_SOCKET)
		{
			{
				std::lock_guard<std::mutex> lock(g_mutex);
				if (g_stopping)
					return;
			}
			Sleep(50);
			continue;
		}

		DWORD timeout = recv_timeout_ms;
		setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
		serve(client);
		shutdown(client, SD_SEND);
		closesocket(client);
	}
}
}

bool is_builtin_tool(const std::string& name)
{
	return find_builtin(name) != nullptr;
}
}

void mcp_startup()
{
	using namespace mcp;

	if (g_running || !has_param(Core.Params, "-mcp"))
		return;

	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		Msg("! [mcp] WSAStartup failed");
		return;
	}

	u16 port = port_from_params();
	SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (s != INVALID_SOCKET)
	{
		BOOL exclusive = TRUE;
		setsockopt(s, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char*>(&exclusive), sizeof(exclusive));
	}

	sockaddr_in addr = {};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (s == INVALID_SOCKET || bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR || listen(s, 8) == SOCKET_ERROR)
	{
		Msg("! [mcp] cannot listen on 127.0.0.1:%u (WSA error %d)", u32(port), WSAGetLastError());
		if (s != INVALID_SOCKET)
			closesocket(s);
		WSACleanup();
		return;
	}

	{
		std::lock_guard<std::mutex> lock(g_mutex);
		g_stopping = false;
		g_status = Status();
	}
	g_listen = s;
	Device.seqFrame.Add(&g_pump, REG_PRIORITY_LOW);
	g_thread = std::thread(server_loop, s);
	g_running = true;
	Msg("* [mcp] listening on http://127.0.0.1:%u/mcp", u32(port));
}

void mcp_shutdown()
{
	using namespace mcp;

	if (!g_running)
		return;

	Device.seqFrame.Remove(&g_pump);
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		g_stopping = true;
	}
	g_cv.notify_all();
	closesocket(g_listen);
	g_listen = INVALID_SOCKET;
	g_thread.join();
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		g_queue.clear();
	}
	WSACleanup();
	g_running = false;
}
