#pragma once

namespace perception
{
	struct dds_info
	{
		u32 width, height, mips;
		u32 fourcc; // 0 for uncompressed
		u32 bits;   // bpp for uncompressed
		u32 amask;
	};

	bool dds_probe(const u8* data, u32 size, dds_info& out);

	// Share of mip-0 texels drawn at the alpha cutoff (alpha >= alpha_ref, 0..1) in each cell of a
	// res x res grid over the whole image, as 0..255, row-major. DXT1 is counted from its index bits.
	// A cell spanning more than 8 blocks per axis reads a strided 4 x 4 subset of them. false if
	// unparseable, or if res is not a power of two no larger than the image.
	bool dds_coverage_grid(const u8* data, u32 size, u32 res, float alpha_ref, xr_vector<u8>& out);

	// Point lookups of mip-0 alpha. Points into the caller's buffer, which must outlive the view.
	class dds_alpha_view
	{
	public:
		dds_alpha_view() : m_lvl(0) {}

		bool open(const u8* data, u32 size);
		bool valid() const { return 0 != m_lvl; }
		u32 width() const { return m_info.width; }
		u32 height() const { return m_info.height; }

		// 0..255 at the texel containing (u, v); UVs wrap.
		u32 alpha(float u, float v) const;

	private:
		dds_info m_info;
		const u8* m_lvl;
	};
}
