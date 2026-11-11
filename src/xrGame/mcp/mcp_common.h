#pragma once

#include "3rd party/json11/json11.hpp"

#include <string>

namespace mcp
{
using json11::Json;

struct ToolResult
{
	Json::array content;
	bool is_error = false;

	void text(const std::string& s);
	void fail(const std::string& message);
};

std::string get_str(const Json& object, const char* key, const char* fallback = "");
long long get_int(const Json& object, const char* key, long long fallback);

// invalid UTF-8 in strings is replaced with U+FFFD
std::string dump(const Json& j);

// game strings are cp1251; text already in valid UTF-8 passes through unchanged
std::string to_utf8(const char* s, size_t n);
std::string to_utf8(const char* s);
std::string to_game(const std::string& utf8);

std::string base64(const u8* data, size_t n);

size_t log_mark();
Json::array log_since(size_t mark);

bool is_builtin_tool(const std::string& name);

// main thread only
ToolResult lua_run(const std::string& code);
ToolResult lua_call_tool(const std::string& name, const Json& args);

// any thread
Json::array script_tool_list();
bool script_tool_exists(const std::string& name);
}
