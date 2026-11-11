#include "stdafx.h"
#include "mcp_common.h"

namespace mcp
{
namespace
{
constexpr UINT game_codepage = 1251;
constexpr size_t max_captured_lines = 200;

bool is_ascii(const char* s, size_t n)
{
	for (size_t i = 0; i < n; ++i)
		if (static_cast<u8>(s[i]) >= 0x80)
			return false;
	return true;
}

bool is_utf8(const char* s, size_t n)
{
	return MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, s, static_cast<int>(n), nullptr, 0) > 0;
}

std::string wide_to_utf8(const std::wstring& wide)
{
	int len = WideCharToMultiByte(CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()), nullptr, 0, nullptr, nullptr);
	std::string out(len, '\0');
	WideCharToMultiByte(CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()), &out[0], len, nullptr, nullptr);
	return out;
}

std::wstring to_wide(UINT codepage, const char* s, size_t n)
{
	int len = MultiByteToWideChar(codepage, 0, s, static_cast<int>(n), nullptr, 0);
	std::wstring wide(len, L'\0');
	MultiByteToWideChar(codepage, 0, s, static_cast<int>(n), &wide[0], len);
	return wide;
}
}

void ToolResult::text(const std::string& s)
{
	content.push_back(Json::object{{"type", "text"}, {"text", s}});
}

void ToolResult::fail(const std::string& message)
{
	text(message);
	is_error = true;
}

std::string get_str(const Json& object, const char* key, const char* fallback)
{
	const Json& value = object[key];
	return value.is_string() ? value.string_value() : std::string(fallback);
}

long long get_int(const Json& object, const char* key, long long fallback)
{
	const Json& value = object[key];
	if (!value.is_number())
		return fallback;
	double d = value.number_value();
	if (!(d > -9.0e18 && d < 9.0e18))
		return fallback;
	return static_cast<long long>(d);
}

std::string dump(const Json& j)
{
	std::string out = j.dump();
	if (is_ascii(out.data(), out.size()) || is_utf8(out.data(), out.size()))
		return out;
	// without MB_ERR_INVALID_CHARS, invalid sequences decode to U+FFFD
	return wide_to_utf8(to_wide(CP_UTF8, out.data(), out.size()));
}

std::string to_utf8(const char* s, size_t n)
{
	if (!s || !n)
		return std::string();
	if (is_ascii(s, n) || is_utf8(s, n))
		return std::string(s, n);
	return wide_to_utf8(to_wide(game_codepage, s, n));
}

std::string to_utf8(const char* s)
{
	return to_utf8(s, s ? xr_strlen(s) : 0);
}

std::string to_game(const std::string& utf8)
{
	if (is_ascii(utf8.data(), utf8.size()))
		return utf8;

	int wide_len = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, utf8.data(), static_cast<int>(utf8.size()), nullptr, 0);
	if (wide_len <= 0)
		return utf8;
	std::wstring wide(wide_len, L'\0');
	MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), &wide[0], wide_len);

	BOOL lossy = FALSE;
	int len = WideCharToMultiByte(game_codepage, WC_NO_BEST_FIT_CHARS, wide.data(), wide_len, nullptr, 0, nullptr, &lossy);
	if (len <= 0 || lossy)
		return utf8;
	std::string out(len, '\0');
	WideCharToMultiByte(game_codepage, WC_NO_BEST_FIT_CHARS, wide.data(), wide_len, &out[0], len, nullptr, nullptr);
	return out;
}

std::string base64(const u8* data, size_t n)
{
	static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

	std::string out;
	out.reserve((n + 2) / 3 * 4);

	size_t i = 0;
	for (; i + 2 < n; i += 3)
	{
		u32 v = (u32(data[i]) << 16) | (u32(data[i + 1]) << 8) | u32(data[i + 2]);
		out += alphabet[(v >> 18) & 63];
		out += alphabet[(v >> 12) & 63];
		out += alphabet[(v >> 6) & 63];
		out += alphabet[v & 63];
	}
	if (i < n)
	{
		u32 v = u32(data[i]) << 16;
		if (i + 1 < n)
			v |= u32(data[i + 1]) << 8;
		out += alphabet[(v >> 18) & 63];
		out += alphabet[(v >> 12) & 63];
		out += i + 1 < n ? alphabet[(v >> 6) & 63] : '=';
		out += '=';
	}
	return out;
}

size_t log_mark()
{
	return LogLineCount();
}

Json::array log_since(size_t mark)
{
	size_t end = LogLineCount();
	if (end <= mark)
		return Json::array();

	size_t skipped = end - mark > max_captured_lines ? end - mark - max_captured_lines : 0;
	xr_vector<xr_string> lines;
	LogCopyLines(mark + skipped, end, lines);

	Json::array out;
	if (skipped)
		out.push_back("<" + std::to_string(skipped) + " earlier lines omitted>");
	for (const xr_string& line : lines)
		out.push_back(to_utf8(line.c_str(), line.size()));
	return out;
}
}
