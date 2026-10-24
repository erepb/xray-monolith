#pragma once

// Identity of a compiled shader: what its source was, and which defines it was built with.
//
// Both halves are already computed inside every renderer's shader_compile and then discarded:
//   sh_name    - encodes the value of every preprocessor define (r4.cpp:1308-1954)
//   source_crc - getShaderSourceCrc32 (xrCore/ShaderSourceCRC.cpp:85), which recursively folds in
//                every #include resolved through the VFS, so a mod overriding a header changes it
//                and a file nothing includes contributes nothing.
//
// This is provenance, not a cache key. It answers "which shader produced this measurement?" for the
// log and the debug report. It deliberately does not answer "may I reuse the previous measurement?",
// because it cannot see runtime constants the shader reads - ps_ssfx_fog is written from an MCM
// slider and changes the fog without touching source or defines.

namespace shader_fingerprint
{
	void record(LPCSTR name, u32 source_crc, LPCSTR defines);
	bool get(LPCSTR name, u32& source_crc, u32& defines_hash);
}
