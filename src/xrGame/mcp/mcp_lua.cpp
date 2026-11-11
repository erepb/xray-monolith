#include "pch_script.h"
#include "mcp_lua.h"
#include "mcp_common.h"
#include "../ai_space.h"
#include "script_engine.h"

#include <chrono>
#include <cmath>
#include <mutex>
#include <unordered_set>
#include <vector>

namespace mcp
{
namespace
{
constexpr int result_depth = 6;
constexpr int schema_depth = 32;
constexpr size_t max_items = 1000;
constexpr int max_arg_depth = 32;
constexpr size_t max_tool_name = 64;

struct ScriptTool
{
	std::string name;
	std::string description;
	Json schema;
};

// metadata only; the functions live in the Lua state's store and die with it
std::mutex g_tools_mutex;
std::vector<ScriptTool> g_tools;

std::vector<std::string>* g_output = nullptr;

// Everything kept in a Lua state lives in one table at registry[&g_store_key]. Never use luaL_ref:
// the state is recreated at the same address on load and stale refs get released into the new one.
char g_store_key;

const char probe_source[] = R"(
local pcall, type = pcall, type
return function(v)
	local ok, id, name, section = pcall(function() return v:id(), v:name(), v:section() end)
	if ok and type(id) == "number" then return { id = id, name = name, section = section } end
	ok, id, name, section = pcall(function() return v.id, v:name(), v:section_name() end)
	if ok and type(id) == "number" then return { id = id, name = name, section = section } end
	local x, y, z
	ok, x, y, z = pcall(function() return v.x, v.y, v.z end)
	if ok and type(x) == "number" and type(y) == "number" and type(z) == "number" then return { x = x, y = y, z = z } end
end
)";

using Seen = std::unordered_set<const void*>;

void clear_tools()
{
	std::lock_guard<std::mutex> lock(g_tools_mutex);
	g_tools.clear();
}

lua_State* current_state()
{
	return ai().script_engine().lua();
}

void push_store(lua_State* L)
{
	lua_pushlightuserdata(L, &g_store_key);
	lua_rawget(L, LUA_REGISTRYINDEX);
	if (lua_istable(L, -1))
		return;

	lua_pop(L, 1);
	lua_newtable(L);
	lua_pushlightuserdata(L, &g_store_key);
	lua_pushvalue(L, -2);
	lua_rawset(L, LUA_REGISTRYINDEX);
}

// pushes store[key]; nil when absent
void push_stored(lua_State* L, const char* key)
{
	push_store(L);
	lua_pushstring(L, key);
	lua_rawget(L, -2);
	lua_remove(L, -2);
}

// pops the value on top into store[key]
void store_value(lua_State* L, const char* key)
{
	push_store(L);
	lua_pushstring(L, key);
	lua_pushvalue(L, -3);
	lua_rawset(L, -3);
	lua_pop(L, 2);
}

int abs_index(lua_State* L, int idx)
{
	return idx > 0 || idx <= LUA_REGISTRYINDEX ? idx : lua_gettop(L) + idx + 1;
}

std::string safe_tostring(lua_State* L, int idx)
{
	idx = abs_index(L, idx);
	std::string out = "?";
	lua_getglobal(L, "tostring");
	lua_pushvalue(L, idx);
	if (lua_pcall(L, 1, 1, 0) == 0)
	{
		size_t len = 0;
		const char* s = lua_tolstring(L, -1, &len);
		if (s)
			out = to_utf8(s, len);
	}
	lua_pop(L, 1);
	return out;
}

std::string error_message(lua_State* L)
{
	size_t len = 0;
	const char* s = lua_tolstring(L, -1, &len);
	return s ? to_utf8(s, len) : std::string("unknown error");
}

int l_traceback(lua_State* L)
{
	const char* msg = lua_tostring(L, 1);
	if (!msg)
	{
		lua_getglobal(L, "tostring");
		lua_pushvalue(L, 1);
		lua_call(L, 1, 1);
		msg = lua_tostring(L, -1);
	}
	luaL_traceback(L, L, msg, 1);
	return 1;
}

int l_print(lua_State* L)
{
	int n = lua_gettop(L);
	std::string line;
	lua_getglobal(L, "tostring");
	for (int i = 1; i <= n; ++i)
	{
		lua_pushvalue(L, -1);
		lua_pushvalue(L, i);
		lua_call(L, 1, 1);
		size_t len = 0;
		const char* s = lua_tolstring(L, -1, &len);
		if (i > 1)
			line += '\t';
		if (s)
			line.append(s, len);
		lua_pop(L, 1);
	}
	lua_pop(L, 1);

	if (g_output)
		g_output->push_back(to_utf8(line.data(), line.size()));
	else
		Msg("%s", line.c_str());
	return 0;
}

void push_env(lua_State* L)
{
	push_stored(L, "env");
	if (lua_istable(L, -1))
		return;

	lua_pop(L, 1);
	lua_newtable(L);
	lua_newtable(L);
	lua_pushvalue(L, LUA_GLOBALSINDEX);
	lua_setfield(L, -2, "__index");
	lua_setmetatable(L, -2);
	lua_pushcfunction(L, l_print);
	lua_setfield(L, -2, "print");
	lua_pushvalue(L, -1);
	store_value(L, "env");
}

void push_probe(lua_State* L)
{
	push_stored(L, "probe");
	if (lua_isfunction(L, -1))
		return;

	lua_pop(L, 1);
	if (luaL_loadbuffer(L, probe_source, sizeof(probe_source) - 1, "=mcp_probe") != 0 || lua_pcall(L, 0, 1, 0) != 0)
	{
		Msg("! [mcp] %s", lua_tostring(L, -1));
		lua_pop(L, 1);
		lua_pushnil(L);
		return;
	}
	lua_pushvalue(L, -1);
	store_value(L, "probe");
}

void push_tools_table(lua_State* L)
{
	push_stored(L, "tools");
	if (lua_istable(L, -1))
		return;

	lua_pop(L, 1);
	lua_newtable(L);
	lua_pushvalue(L, -1);
	store_value(L, "tools");
}

Json to_json(lua_State* L, int idx, int depth, int max_depth, Seen& seen);

Json number_to_json(lua_Number d)
{
	if (d != d)
		return "nan";
	if (d == HUGE_VAL)
		return "inf";
	if (d == -HUGE_VAL)
		return "-inf";
	return d;
}

std::string key_string(lua_State* L, int idx)
{
	switch (lua_type(L, idx))
	{
	case LUA_TSTRING:
		{
			size_t len = 0;
			const char* s = lua_tolstring(L, idx, &len);
			return to_utf8(s, len);
		}
	case LUA_TNUMBER:
		{
			char buf[64];
			xr_sprintf(buf, sizeof(buf), "%.14g", lua_tonumber(L, idx));
			return buf;
		}
	case LUA_TBOOLEAN:
		return lua_toboolean(L, idx) ? "true" : "false";
	default:
		return safe_tostring(L, idx);
	}
}

Json table_to_json(lua_State* L, int idx, int depth, int max_depth, Seen& seen)
{
	const void* ptr = lua_topointer(L, idx);
	if (seen.count(ptr))
		return "<cycle>";
	if (depth >= max_depth)
		return "<table>";
	seen.insert(ptr);

	size_t length = lua_objlen(L, idx);
	size_t count = 0;
	lua_pushnil(L);
	while (lua_next(L, idx))
	{
		++count;
		lua_pop(L, 1);
	}

	Json out;
	if (length > 0 && count == length)
	{
		Json::array items;
		for (size_t i = 1; i <= length && i <= max_items; ++i)
		{
			lua_rawgeti(L, idx, static_cast<int>(i));
			items.push_back(to_json(L, -1, depth + 1, max_depth, seen));
			lua_pop(L, 1);
		}
		if (length > max_items)
			items.push_back("<+" + std::to_string(length - max_items) + " more>");
		out = Json(std::move(items));
	}
	else
	{
		Json::object fields;
		size_t taken = 0;
		lua_pushnil(L);
		while (lua_next(L, idx))
		{
			if (taken == max_items)
			{
				fields["..."] = "<+" + std::to_string(count - max_items) + " more>";
				lua_pop(L, 2);
				break;
			}
			Json value = to_json(L, -1, depth + 1, max_depth, seen);
			fields[key_string(L, -2)] = std::move(value);
			++taken;
			lua_pop(L, 1);
		}
		out = Json(std::move(fields));
	}

	seen.erase(ptr);
	return out;
}

Json userdata_to_json(lua_State* L, int idx, int depth, int max_depth, Seen& seen)
{
	push_probe(L);
	if (lua_isfunction(L, -1))
	{
		lua_pushvalue(L, idx);
		if (lua_pcall(L, 1, 1, 0) == 0 && lua_istable(L, -1))
		{
			Json out = table_to_json(L, lua_gettop(L), depth, max_depth, seen);
			lua_pop(L, 1);
			return out;
		}
	}
	lua_pop(L, 1);
	return safe_tostring(L, idx);
}

Json to_json(lua_State* L, int idx, int depth, int max_depth, Seen& seen)
{
	idx = abs_index(L, idx);
	switch (lua_type(L, idx))
	{
	case LUA_TNIL:
		return nullptr;
	case LUA_TBOOLEAN:
		return lua_toboolean(L, idx) != 0;
	case LUA_TNUMBER:
		return number_to_json(lua_tonumber(L, idx));
	case LUA_TSTRING:
		{
			size_t len = 0;
			const char* s = lua_tolstring(L, idx, &len);
			return to_utf8(s, len);
		}
	case LUA_TTABLE:
		return table_to_json(L, idx, depth, max_depth, seen);
	case LUA_TUSERDATA:
	case LUA_TLIGHTUSERDATA:
		return userdata_to_json(L, idx, depth, max_depth, seen);
	default:
		return "<" + std::string(lua_typename(L, lua_type(L, idx))) + ">";
	}
}

Json results_to_json(lua_State* L, int first)
{
	int last = lua_gettop(L);
	Seen seen;
	if (last < first)
		return Json();
	if (last == first)
		return to_json(L, first, 0, result_depth, seen);

	Json::array out;
	for (int i = first; i <= last; ++i)
		out.push_back(to_json(L, i, 0, result_depth, seen));
	return Json(std::move(out));
}

void push_json(lua_State* L, const Json& j, int depth)
{
	if (depth > max_arg_depth)
	{
		lua_pushnil(L);
		return;
	}

	switch (j.type())
	{
	case Json::BOOL:
		lua_pushboolean(L, j.bool_value());
		break;
	case Json::NUMBER:
		lua_pushnumber(L, j.number_value());
		break;
	case Json::STRING:
		{
			std::string s = to_game(j.string_value());
			lua_pushlstring(L, s.data(), s.size());
		}
		break;
	case Json::ARRAY:
		{
			const Json::array& items = j.array_items();
			lua_createtable(L, static_cast<int>(items.size()), 0);
			int i = 1;
			for (const Json& item : items)
			{
				push_json(L, item, depth + 1);
				lua_rawseti(L, -2, i++);
			}
		}
		break;
	case Json::OBJECT:
		{
			const Json::object& fields = j.object_items();
			lua_createtable(L, 0, static_cast<int>(fields.size()));
			for (const auto& field : fields)
			{
				std::string key = to_game(field.first);
				push_json(L, field.second, depth + 1);
				lua_setfield(L, -2, key.c_str());
			}
		}
		break;
	default:
		lua_pushnil(L);
		break;
	}
}

bool valid_tool_name(const std::string& name)
{
	if (name.empty() || name.size() > max_tool_name)
		return false;
	for (char c : name)
		if (!isalnum(static_cast<u8>(c)) && c != '_' && c != '-' && c != '.')
			return false;
	return true;
}

// mcp.register_tool(name, description, schema, fn): fn(args) runs on the main thread, its return value is the tool result
int l_register_tool(lua_State* L)
{
	size_t name_len = 0;
	const char* name = luaL_checklstring(L, 1, &name_len);
	const char* description = luaL_checkstring(L, 2);
	luaL_checktype(L, 3, LUA_TTABLE);
	luaL_checktype(L, 4, LUA_TFUNCTION);

	std::string tool(name, name_len);
	if (!valid_tool_name(tool))
		return luaL_error(L, "mcp.register_tool: invalid name '%s' (1-64 letters, digits, '_', '-', '.')", name);
	if (is_builtin_tool(tool))
		return luaL_error(L, "mcp.register_tool: '%s' is a built-in tool", name);

	lua_checkstack(L, 4 * schema_depth);
	Seen seen;
	Json schema = to_json(L, 3, 0, schema_depth, seen);
	if (!schema.is_object() || get_str(schema, "type") != "object")
		return luaL_error(L, "mcp.register_tool: schema of '%s' must be a table with type = \"object\"", name);
	const Json& required = schema["required"];
	if (required.is_object() && required.object_items().empty())
	{
		Json::object fields = schema.object_items();
		fields["required"] = Json::array();
		schema = Json(std::move(fields));
	}

	push_tools_table(L);
	lua_pushlstring(L, tool.data(), tool.size());
	lua_pushvalue(L, 4);
	lua_rawset(L, -3);
	lua_pop(L, 1);

	std::lock_guard<std::mutex> lock(g_tools_mutex);
	for (ScriptTool& existing : g_tools)
	{
		if (existing.name != tool)
			continue;
		existing.description = to_utf8(description);
		existing.schema = std::move(schema);
		return 0;
	}
	g_tools.push_back(ScriptTool{tool, to_utf8(description), std::move(schema)});
	return 0;
}
}

ToolResult lua_run(const std::string& code_utf8)
{
	ToolResult r;
	if (code_utf8.empty())
	{
		r.fail("code is empty");
		return r;
	}
	lua_State* L = current_state();
	if (!L)
	{
		r.fail("the script engine is not initialized");
		return r;
	}

	int top = lua_gettop(L);
	lua_checkstack(L, 4 * result_depth + 16);

	std::vector<std::string> output;
	g_output = &output;
	size_t mark = log_mark();
	auto started = std::chrono::steady_clock::now();

	std::string code = to_game(code_utf8);
	lua_pushcfunction(L, l_traceback);
	int handler = lua_gettop(L);

	std::string expression = "return " + code;
	int status = luaL_loadbuffer(L, expression.data(), expression.size(), "=mcp");
	if (status != 0)
	{
		lua_pop(L, 1);
		status = luaL_loadbuffer(L, code.data(), code.size(), "=mcp");
	}

	Json::object out;
	if (status == 0)
	{
		push_env(L);
		lua_setfenv(L, -2);
		status = lua_pcall(L, 0, LUA_MULTRET, handler);
		if (status == 0)
			out["result"] = results_to_json(L, handler + 1);
	}
	if (status != 0)
		out["error"] = error_message(L);

	g_output = nullptr;
	lua_settop(L, top);

	if (!output.empty())
		out["output"] = Json(Json::array(output.begin(), output.end()));
	Json::array log = log_since(mark);
	if (!log.empty())
		out["log"] = Json(std::move(log));
	out["ms"] = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started).count());

	r.text(dump(Json(std::move(out))));
	r.is_error = status != 0;
	return r;
}

ToolResult lua_call_tool(const std::string& name, const Json& args)
{
	ToolResult r;
	lua_State* L = current_state();
	if (!L || !script_tool_exists(name))
	{
		r.fail("tool '" + name + "' is no longer registered (the script state was reloaded)");
		return r;
	}

	int top = lua_gettop(L);
	lua_checkstack(L, 2 * max_arg_depth + 4 * result_depth + 16);

	lua_pushcfunction(L, l_traceback);
	int handler = lua_gettop(L);
	push_tools_table(L);
	lua_pushlstring(L, name.data(), name.size());
	lua_rawget(L, -2);
	lua_remove(L, -2);
	if (!lua_isfunction(L, -1))
	{
		lua_settop(L, top);
		r.fail("tool '" + name + "' is no longer registered (the script state was reloaded)");
		return r;
	}
	push_json(L, args, 0);

	if (lua_pcall(L, 1, LUA_MULTRET, handler) == 0)
	{
		Json result = results_to_json(L, handler + 1);
		r.text(result.is_string() ? result.string_value() : dump(result));
	}
	else
		r.fail(error_message(L));

	lua_settop(L, top);
	return r;
}

Json::array script_tool_list()
{
	std::lock_guard<std::mutex> lock(g_tools_mutex);
	Json::array out;
	for (const ScriptTool& tool : g_tools)
		out.push_back(Json::object{{"name", tool.name}, {"description", tool.description}, {"inputSchema", tool.schema}});
	return out;
}

bool script_tool_exists(const std::string& name)
{
	std::lock_guard<std::mutex> lock(g_tools_mutex);
	for (const ScriptTool& tool : g_tools)
		if (tool.name == name)
			return true;
	return false;
}
}

void mcp_registrator::script_register(lua_State* L)
{
	mcp::clear_tools();

	lua_newtable(L);
	lua_pushcfunction(L, mcp::l_register_tool);
	lua_setfield(L, -2, "register_tool");
	lua_setglobal(L, "mcp");
}
