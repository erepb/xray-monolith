#include "stdafx.h"
#include "UIFrameLineWnd.h"
#include "UITextureMaster.h"

CUIFrameLineWnd::CUIFrameLineWnd()
	: bHorizontal(true),
	  m_bTextureVisible(false),
	  m_cap_scaled(false)
{
	m_texture_color = color_argb(255, 255, 255, 255);
	for (int k = 0; k < flMax; ++k)
		m_tex_rect[k].set(0.0f, 0.0f, 0.0f, 0.0f);
}

void CUIFrameLineWnd::InitFrameLineWnd(LPCSTR base_name, Fvector2 pos, Fvector2 size, bool horizontal)
{
	InitFrameLineWnd(pos, size, horizontal);
	InitTexture(base_name, "hud\\default");
}

void CUIFrameLineWnd::InitFrameLineWnd(Fvector2 pos, Fvector2 size, bool horizontal)
{
	inherited::SetWndPos(pos);
	inherited::SetWndSize(size);

	bHorizontal = horizontal;
}

void CUIFrameLineWnd::InitTexture(LPCSTR texture, LPCSTR sh_name)
{
	m_bTextureVisible = true;
	m_texture_name = texture;
	// A missing slice leaves its rect untouched by CUITextureMaster::InitTexture, so pre-zero to a known
	// state: the asserts below then pass on all-missing (0==0) and DeriveCapsIfMissing can detect it.
	for (int k = 0; k < flMax; ++k)
		m_tex_rect[k].set(0.0f, 0.0f, 0.0f, 0.0f);
	string256 buf;
	CUITextureMaster::InitTexture(strconcat(sizeof(buf), buf, texture, "_back"), sh_name, m_shader, m_tex_rect[flBack]);
	CUITextureMaster::InitTexture(strconcat(sizeof(buf), buf, texture, "_b"), sh_name, m_shader, m_tex_rect[flFirst]);
	m_cap_overlap = CUITextureMaster::GetTextureOverlap(buf);
	CUITextureMaster::InitTexture(strconcat(sizeof(buf), buf, texture, "_e"), sh_name, m_shader, m_tex_rect[flSecond]);
	if (bHorizontal)
	{
		R_ASSERT2(fsimilar(m_tex_rect[flFirst].height(), m_tex_rect[flSecond].height()), texture);
		R_ASSERT2(fsimilar(m_tex_rect[flFirst].height(), m_tex_rect[flBack].height()), texture);
	}
	else
	{
		R_ASSERT2(fsimilar(m_tex_rect[flFirst].width(), m_tex_rect[flSecond].width()), texture);
		R_ASSERT2(fsimilar(m_tex_rect[flFirst].width(), m_tex_rect[flBack].width()), texture);
	}

	if (m_cap_scaled)
		DeriveCapsIfMissing();
}

void CUIFrameLineWnd::SetCapScaled(bool b)
{
	m_cap_scaled = b;
	if (b)
		DeriveCapsIfMissing();
}

void CUIFrameLineWnd::DeriveCapsIfMissing()
{
	if (m_tex_rect[flFirst].width() > 0.0f) // authored _b cap present -> use the shipped slices as-is
		return;
	// The per-state _b/_back/_e lookups above all missed, so m_shader was left bound to a non-existent
	// "<id>_e" file (the last miss) -> the caps would draw black. Rebind shader AND rect to the base state
	// atlas id (the un-suffixed id, e.g. ui_inGame2_pda_button_e), the synthesis source. A miss here --
	// absent id, or a raw file-path texture not in any descr -- yields a zero rect and bails.
	Frect base;
	base.set(0.0f, 0.0f, 0.0f, 0.0f);
	CUITextureMaster::InitTexture(m_texture_name, "hud\\default", m_shader, base);
	if (base.width() <= 0.0f || base.height() <= 0.0f)
		return; // nothing to derive from -> leave empty (draws nothing), never garbage

	// Square-by-height caps carved from the base rect's ends; a 1-texel centre column tiles the middle. The
	// carved caps include whatever slanted alpha the art has at its ends, so interlock the tabs by the cap
	// width -- a parallelogram art then reads as a continuous strip instead of separated rectangles.
	const float cap = _min(base.height(), base.width() * 0.5f);
	m_tex_rect[flFirst].set(base.x1, base.y1, base.x1 + cap, base.y2);   // _b  left cap
	m_tex_rect[flSecond].set(base.x2 - cap, base.y1, base.x2, base.y2);  // _e  right cap
	const float midx = (base.x1 + base.x2) * 0.5f;
	m_tex_rect[flBack].set(midx - 0.5f, base.y1, midx + 0.5f, base.y2);  // _back centre column
	m_cap_overlap = cap;
}

void CUIFrameLineWnd::Draw()
{
	if (m_bTextureVisible)
		DrawElements();

	inherited::Draw();
}

static Fvector2 pt_offset = {-0.5f, -0.5f};

void draw_rect(Fvector2 LTp, Fvector2 RBp, Fvector2 LTt, Fvector2 RBt, u32 clr, Fvector2 const& ts)
{
	UI().AlignPixel(LTp.x);
	UI().AlignPixel(LTp.y);
	LTp.add(pt_offset);
	UI().AlignPixel(RBp.x);
	UI().AlignPixel(RBp.y);
	RBp.add(pt_offset);
	LTt.div(ts);
	RBt.div(ts);

	// Frame-lines push their vertices directly, so a custom clip must be applied here in software
	if (UI().HasCustomClip())
	{
		sPoly2D S;
		S.resize(4);
		S[0].set(LTp.x, LTp.y, LTt.x, LTt.y);
		S[1].set(RBp.x, LTp.y, RBt.x, LTt.y);
		S[2].set(RBp.x, RBp.y, RBt.x, RBt.y);
		S[3].set(LTp.x, RBp.y, LTt.x, RBt.y);
		sPoly2D D;
		sPoly2D* R = UI().ActiveClipFrustum().ClipPoly(S, D);
		if (R && R->size())
		{
			for (u32 k = 0; k < R->size() - 2; ++k)
			{
				UIRender->PushPoint((*R)[0].pt.x, (*R)[0].pt.y, 0, clr, (*R)[0].uv.x, (*R)[0].uv.y);
				UIRender->PushPoint((*R)[k + 1].pt.x, (*R)[k + 1].pt.y, 0, clr, (*R)[k + 1].uv.x, (*R)[k + 1].uv.y);
				UIRender->PushPoint((*R)[k + 2].pt.x, (*R)[k + 2].pt.y, 0, clr, (*R)[k + 2].uv.x, (*R)[k + 2].uv.y);
			}
		}
		return;
	}

	UIRender->PushPoint(LTp.x, LTp.y, 0, clr, LTt.x, LTt.y);
	UIRender->PushPoint(RBp.x, RBp.y, 0, clr, RBt.x, RBt.y);
	UIRender->PushPoint(LTp.x, RBp.y, 0, clr, LTt.x, RBt.y);

	UIRender->PushPoint(LTp.x, LTp.y, 0, clr, LTt.x, LTt.y);
	UIRender->PushPoint(RBp.x, LTp.y, 0, clr, RBt.x, LTt.y);
	UIRender->PushPoint(RBp.x, RBp.y, 0, clr, RBt.x, RBt.y);
}

void CUIFrameLineWnd::DrawElements()
{
	UIRender->SetShader(*m_shader);

	Fvector2 ts;
	UIRender->GetActiveTextureResolution(ts);

	Frect rect;
	GetAbsoluteRect(rect);
	Frect ui_rect = rect;
	UI().ClientToScreenScaled(rect.lt);
	UI().ClientToScreenScaled(rect.rb);

	float scale_cap = 1.0f;
	if (m_cap_scaled && bHorizontal && ui_rect.width() > 0.0f && m_tex_rect[flFirst].height() > 0.0f)
	{
		const float scale_tex = ui_rect.height() / m_tex_rect[flFirst].height();
		const float scale_res = rect.width()     / ui_rect.width();
		scale_cap = scale_tex * scale_res;
	}

	float back_len = 0.0f;
	u32 prim_count = 6 * 2; //first&second
	if (bHorizontal)
	{
		back_len = rect.width() - (m_tex_rect[flFirst].width() + m_tex_rect[flSecond].width()) * scale_cap;
		if (back_len < 0.0f)
			rect.x2 -= back_len;

		if (back_len > 0.0f)
			prim_count += 6 * iCeil(back_len / m_tex_rect[flBack].width());
	}
	else
	{
		back_len = rect.height() - m_tex_rect[flFirst].height() - m_tex_rect[flSecond].height();
		if (back_len < 0)
			rect.y2 -= back_len;

		if (back_len > 0.0f)
			prim_count += 6 * iCeil(back_len / m_tex_rect[flBack].height());
	}

	if (UI().HasCustomClip())
		prim_count = (prim_count / 6) * UI().ActiveClipFrustum().ClipBudget(4);

	UIRender->StartPrimitive(prim_count, IUIRender::ptTriList, UI().m_currentPointType);

	for (int i = 0; i < flMax; ++i)
	{
		Fvector2 LTt, RBt;
		Fvector2 LTp, RBp;
		int counter = 0;

		while (inc_pos(rect, counter, i, LTp, RBp, LTt, RBt, scale_cap))
		{
			draw_rect(LTp, RBp, LTt, RBt, m_texture_color, ts);
			++counter;
		};
	}
	UIRender->FlushPrimitive();
}


bool CUIFrameLineWnd::inc_pos(Frect& rect, int counter, int i, Fvector2& LTp, Fvector2& RBp, Fvector2& LTt,
                              Fvector2& RBt, float scale_cap)
{
	if (i == flFirst || i == flSecond)
	{
		if (counter != 0) return false;

		LTt = m_tex_rect[i].lt;
		RBt = m_tex_rect[i].rb;

		LTp = rect.lt;

		RBp = rect.lt;
		RBp.x += m_tex_rect[i].width() * scale_cap;
		RBp.y += m_tex_rect[i].height();
	}
	else //i==flBack
	{
		if ((bHorizontal && rect.lt.x + m_tex_rect[flSecond].width() * scale_cap + EPS_L >= rect.rb.x) ||
			(!bHorizontal && rect.lt.y + m_tex_rect[flSecond].height() + EPS_L >= rect.rb.y))
			return false;

		LTt = m_tex_rect[i].lt;
		LTp = rect.lt;

		bool b_draw_reminder = (bHorizontal)
			                       ? (rect.lt.x + m_tex_rect[flBack].width() > rect.rb.x - m_tex_rect[flSecond].width() * scale_cap)
			                       : (rect.lt.y + m_tex_rect[flBack].height() > rect.rb.y - m_tex_rect[flSecond].
				                       height());
		if (b_draw_reminder)
		{
			//draw reminder
			float rem_len = (bHorizontal)
				                ? rect.rb.x - m_tex_rect[flSecond].width() * scale_cap - rect.lt.x
				                : rect.rb.y - m_tex_rect[flSecond].height() - rect.lt.y;

			if (bHorizontal)
			{
				RBt.y = m_tex_rect[i].rb.y;
				RBt.x = m_tex_rect[i].lt.x + rem_len;

				RBp = rect.lt;
				RBp.x += rem_len;
				RBp.y += m_tex_rect[i].height();
			}
			else
			{
				RBt.y = m_tex_rect[i].lt.y + rem_len;
				RBt.x = m_tex_rect[i].rb.x;

				RBp = rect.lt;
				RBp.x += m_tex_rect[i].width();
				RBp.y += rem_len;
			}
		}
		else
		{
			//draw full element
			RBt = m_tex_rect[i].rb;

			RBp = rect.lt;
			RBp.x += m_tex_rect[i].width();
			RBp.y += m_tex_rect[i].height();
		}
	}

	//stretch always
	if (bHorizontal)
		RBp.y = rect.rb.y;
	else
		RBp.x = rect.rb.x;

	if (bHorizontal) rect.lt.x = RBp.x;
	else rect.lt.y = RBp.y;
	return true;
}
