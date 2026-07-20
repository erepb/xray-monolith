#pragma once

#include "UIWindow.h"

class CUIFrameLineWnd : public CUIWindow
{
	typedef CUIWindow inherited;
public:
	CUIFrameLineWnd();
	void InitFrameLineWnd(LPCSTR base_name, Fvector2 pos, Fvector2 size, bool horizontal = true);
	void InitFrameLineWnd(Fvector2 pos, Fvector2 size, bool horizontal = true);
	void InitTexture(LPCSTR tex_name, LPCSTR sh_name = "hud\\default");
	virtual void Draw();

	float GetTextureHeight() const { return m_tex_rect[flFirst].height(); }
	float GetBeginCapWidth() const { return m_tex_rect[flFirst].width(); }
	float GetCapOverlap() const { return m_cap_overlap; }
	void SetTextureColor(u32 cl) { m_texture_color = cl; }
	bool IsHorizontal() { return bHorizontal; }
	void SetHorizontal(bool horiz) { bHorizontal = horiz; }
	// When true, caps are scaled to fit the element -- texture->UI by height, then UI->screen for
	// resolution -- so the caps keep their authored shape. When false the cap width is the texture's own
	// pixel width used as-is on screen (never scaled) while its height stretches to fill the element, so
	// the cap's proportions are not preserved. Enabling it also synthesizes the 3-slice from the base
	// state rect when the descr ships no authored _b/_back/_e caps (DeriveCapsIfMissing).
	void SetCapScaled(bool b);
	bool GetCapScaled() const { return m_cap_scaled; }
	LPCSTR GetTextureName() const { return m_texture_name.c_str(); }
protected:
	bool bHorizontal;
	bool inc_pos(Frect& rect, int counter, int i, Fvector2& LTp, Fvector2& RBp, Fvector2& LTt, Fvector2& RBt,
	             float scale_cap);

	enum
	{
		flFirst = 0,
		// Left or top
		flBack,
		// Center texture
		flSecond,
		// Right or bottom
		flMax
	};

	u32 m_texture_color;
	bool m_bTextureVisible;
	bool m_cap_scaled;
	float m_cap_overlap = -1.0f; // begin-cap interlock overlap in atlas texels; <0 => unset
	void DrawElements();
	// Cap-scaled framelines whose descr lacks _b/_back/_e slices: build the 3-slice from the base state
	// rect so the caps keep their shape at any width, without any dedicated cap art in the atlas.
	void DeriveCapsIfMissing();

	ui_shader m_shader;
	Frect m_tex_rect [flMax];
	shared_str m_texture_name;
};
