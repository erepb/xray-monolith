#include "StdAfx.h"

#include "UIRangeBar.h"
#include "UIFrameLineWnd.h"
#include "UIStatic.h"
#include "UITextureMaster.h"
#include "../../xrEngine/xr_input.h"

CUIRangeBar::CUIRangeBar()
	: m_fill(NULL),
	  m_bmin(0.f),
	  m_bmax(1.f),
	  m_step(0.01f),
	  m_drag(-1),
	  m_pending(-1),
	  m_top(0),
	  m_grab_dx(0.f)
{
}

CUIRangeBar::~CUIRangeBar()
{
}

static bool tex_exists(LPCSTR name)
{
	return CUITextureMaster::FindItem(name).file.size() != 0;
}

static void init_knob_states(CUI_IB_Static* k, LPCSTR base)
{
	string_path e, s;
	strconcat(sizeof(e), e, base, "_e");
	k->InitState(S_Enabled, e);

	static const struct { IBState st; LPCSTR suffix; } states[] =
	{
		{S_Disabled, "_d"}, {S_Highlighted, "_h"}, {S_Touched, "_t"},
	};
	for (const auto& it : states)
	{
		strconcat(sizeof(s), s, base, it.suffix);
		k->InitState(it.st, tex_exists(s) ? s : e);
	}
	k->SetCurrentState(S_Enabled);
}

void CUIRangeBar::InitRangeBar(Fvector2 pos, Fvector2 size, int count, LPCSTR track, LPCSTR knob,
                               LPCSTR knob_min, LPCSTR knob_max)
{
	InitIB(pos, size);
	InitState(S_Enabled, track);

	count = _max(1, count);
	m_val.resize(count);
	m_knob.resize(count);

	string_path knob_e;
	strconcat(sizeof(knob_e), knob_e, knob, "_e");
	Fvector2 knob_size;
	knob_size.x = CUITextureMaster::GetTextureWidth(knob_e) * UI().get_current_kx();
	knob_size.y = CUITextureMaster::GetTextureHeight(knob_e);

	for (int r = 0; r < count; ++r)
	{
		LPCSTR base = knob;
		if (r == 0 && knob_min && *knob_min) base = knob_min;
		else if (r == count - 1 && knob_max && *knob_max) base = knob_max;

		CUI_IB_Static* k = xr_new<CUI_IB_Static>();
		k->SetAutoDelete(true);
		k->SetCustomDraw(true);
		AttachChild(k);
		k->InitIB(Fvector2().set(0.f, 0.f), knob_size);
		init_knob_states(k, base);
		m_knob[r] = k;
	}

	for (int r = 0; r < count; ++r)
		m_val[r] = Quantize((count == 1) ? m_bmin + span() / 2.f : m_bmin + span() * r / (count - 1));

	m_top = count - 1;
	Layout();
}

void CUIRangeBar::SetFill(CUIWindow* f, float height)
{
	if (m_fill)
		DetachChild(m_fill);
	m_fill = f;
	f->SetAutoDelete(true);
	f->SetCustomDraw(true);
	AttachChild(f);
	f->SetWndSize(Fvector2().set(0.f, height > 0.f ? height : GetHeight()));
	Layout();
}

void CUIRangeBar::SetFillArt(LPCSTR base, float height)
{
	CUIFrameLineWnd* f = xr_new<CUIFrameLineWnd>();
	f->InitFrameLineWnd(base, Fvector2().set(0.f, 0.f), Fvector2().set(0.f, GetHeight()), true);
	SetFill(f, height);
}

void CUIRangeBar::SetFillColor(u32 argb, float height)
{
	CUIStatic* f = xr_new<CUIStatic>();
	f->InitTexture("ui_inGame2_white_rect");
	f->SetStretchTexture(true);
	f->SetTextureColor(argb);
	SetFill(f, height);
}

// ---- geometry

float CUIRangeBar::knob_w() const
{
	return m_knob.empty() ? 0.f : m_knob[0]->GetWidth();
}

float CUIRangeBar::travel() const
{
	return GetWidth() - knob_w();
}

float CUIRangeBar::span() const
{
	return (m_bmax > m_bmin) ? m_bmax - m_bmin : 1.f;
}

float CUIRangeBar::ValueToX(float v) const
{
	return (v - m_bmin) / span() * travel();
}

float CUIRangeBar::XToValue(float px) const
{
	float t = travel();
	if (t <= 0.f)
		return m_bmin;
	float d = px - knob_w() / 2.f;
	clamp(d, 0.f, t);
	return m_bmin + span() * d / t;
}

float CUIRangeBar::Quantize(float v) const
{
	if (m_bmax <= m_bmin)
		return m_bmin;
	if (m_step > 0.f)
		v = m_bmin + m_step * floorf((v - m_bmin) / m_step + 0.5f);
	clamp(v, m_bmin, m_bmax);
	return v;
}

float CUIRangeBar::ClampToNeighbours(int rank, float v) const
{
	if (rank > 0)
		v = _max(v, m_val[rank - 1]);
	if (rank < GetCount() - 1)
		v = _min(v, m_val[rank + 1]);
	return v;
}

float CUIRangeBar::KnobCenterX(int rank) const
{
	return ValueToX(m_val[rank]) + knob_w() / 2.f;
}

// ---- values

void CUIRangeBar::Requantize()
{
	for (int r = 0; r < GetCount(); ++r)
		m_val[r] = Quantize(m_val[r]);
	for (int r = 1; r < GetCount(); ++r)
		m_val[r] = _max(m_val[r], m_val[r - 1]);
	Layout();
}

void CUIRangeBar::SetBounds(float bmin, float bmax)
{
	m_bmin = bmin;
	m_bmax = bmax;
	Requantize();
}

void CUIRangeBar::SetStep(float step)
{
	m_step = step;
	Requantize();
}

void CUIRangeBar::SetValue(int rank, float v)
{
	if (m_val.empty())
		return;
	clamp(rank, 0, GetCount() - 1);
	m_val[rank] = ClampToNeighbours(rank, Quantize(v));
	Layout();
}

float CUIRangeBar::GetValue(int rank) const
{
	if (m_val.empty())
		return 0.f;
	clamp(rank, 0, GetCount() - 1);
	return m_val[rank];
}

void CUIRangeBar::SetValues(float a, float b)
{
	if (m_val.empty())
		return;
	if (a > b)
		std::swap(a, b);
	if (GetCount() == 1)
	{
		SetValue(0, a);
		return;
	}

	int last = GetCount() - 1;
	m_val[0] = a;
	m_val[last] = b;
	for (int r = 1; r < last; ++r)
		clamp(m_val[r], a, b);
	Requantize();
}

// ---- layout & drawing

void CUIRangeBar::Layout()
{
	if (m_knob.empty())
		return;

	float h = GetHeight();
	for (int r = 0; r < GetCount(); ++r)
	{
		CUI_IB_Static* k = m_knob[r];
		k->SetWndPos(Fvector2().set(ValueToX(m_val[r]), (h - k->GetHeight()) / 2.f));
	}

	if (m_fill)
	{
		float x1 = KnobCenterX(0);
		float x2 = KnobCenterX(GetCount() - 1);
		float fh = m_fill->GetHeight();
		m_fill->SetWndPos(Fvector2().set(x1, (h - fh) / 2.f));
		m_fill->SetWndSize(Fvector2().set(x2 - x1, fh));
	}
}

void CUIRangeBar::UpdateKnobStates()
{
	for (int r = 0; r < GetCount(); ++r)
	{
		CUI_IB_Static* k = m_knob[r];
		IBState st = S_Enabled;
		if (!m_bIsEnabled) st = S_Disabled;
		else if (r == m_drag) st = S_Touched;
		else if (k->CursorOverWindow()) st = S_Highlighted;
		k->SetCurrentState(st);
	}
}

void CUIRangeBar::Enable(bool status)
{
	m_bIsEnabled = status;
	SetCurrentState(m_bIsEnabled ? S_Enabled : S_Disabled);
	if (!m_bIsEnabled)
		EndDrag();
	UpdateKnobStates();
}

void CUIRangeBar::Draw()
{
	inherited::Draw();

	if (m_fill && m_fill->GetWidth() > 0.f)
		m_fill->Draw();

	if (m_knob.empty())
		return;

	for (int r = 0; r < GetCount(); ++r)
		if (r != m_top)
			m_knob[r]->Draw();
	m_knob[m_top]->Draw();
}

void CUIRangeBar::Update()
{
	CUIWindow::Update();

	if ((m_drag >= 0 || m_pending >= 0) && !pInput->iGetAsyncBtnState(0))
		EndDrag();

	UpdateKnobStates();
}

// ---- input

int CUIRangeBar::HitKnob(float x) const
{
	if (m_knob.empty())
		return -1;

	auto hit = [&](int r)
	{
		float l = m_knob[r]->GetWndPos().x;
		return x >= l && x < l + m_knob[r]->GetWidth();
	};

	if (hit(m_top))
		return m_top;
	for (int r = GetCount() - 1; r >= 0; --r)
		if (r != m_top && hit(r))
			return r;
	return -1;
}

void CUIRangeBar::StackOf(int rank, int& lo, int& hi) const
{
	lo = hi = rank;
	while (lo > 0 && fsimilar(m_val[lo - 1], m_val[rank]))
		--lo;
	while (hi < GetCount() - 1 && fsimilar(m_val[hi + 1], m_val[rank]))
		++hi;
}

int CUIRangeBar::NearestKnob(float x) const
{
	int best = 0;
	float best_d = _abs(KnobCenterX(0) - x);
	for (int r = 1; r < GetCount(); ++r)
	{
		float d = _abs(KnobCenterX(r) - x);
		if (d < best_d)
		{
			best = r;
			best_d = d;
		}
	}

	int lo, hi;
	StackOf(best, lo, hi);
	if (lo == hi)
		return best;
	return (x < KnobCenterX(best)) ? lo : hi;
}

void CUIRangeBar::MoveKnob(int rank, float px)
{
	float old = m_val[rank];
	SetValue(rank, XToValue(px));
	if (!fsimilar(old, m_val[rank]) && GetMessageTarget())
		GetMessageTarget()->SendMessage(this, BUTTON_CLICKED, NULL);
}

void CUIRangeBar::EndDrag()
{
	if (m_drag < 0 && m_pending < 0)
		return;
	m_drag = -1;
	m_pending = -1;
	SetCapture(this, false);
}

bool CUIRangeBar::OnMouseAction(float x, float y, EUIMessages mouse_action)
{
	CUIWindow::OnMouseAction(x, y, mouse_action);

	switch (mouse_action)
	{
	case WINDOW_LBUTTON_DOWN:
	case WINDOW_LBUTTON_DB_CLICK:
		{
			EndDrag();
			if (m_val.empty())
				return true;

			int h = HitKnob(x);
			bool on_track = h < 0;
			if (on_track) h = NearestKnob(x);
			m_top = h;
			m_grab_dx = on_track ? 0.f : x - KnobCenterX(h);
			SetCapture(this, true);
			int lo, hi;
			StackOf(h, lo, hi);
			if (on_track || lo == hi) m_drag = h; else m_pending = h;
			if (on_track) MoveKnob(h, x);
		}
		break;

	case WINDOW_MOUSE_MOVE:
		if (!pInput->iGetAsyncBtnState(0))
		{
			EndDrag();
			break;
		}
		if (m_pending >= 0)
		{
			float v = Quantize(XToValue(x - m_grab_dx));
			float sv = m_val[m_pending];
			int lo, hi;
			StackOf(m_pending, lo, hi);
			if (v < sv) m_drag = lo;
			else if (v > sv) m_drag = hi;
			else break;
			m_pending = -1;
			m_top = m_drag;
		}
		if (m_drag >= 0)
			MoveKnob(m_drag, x - m_grab_dx);
		break;

	case WINDOW_LBUTTON_UP:
		EndDrag();
		break;

	default:
		return false;
	}

	return true;
}

void CUIRangeBar::SendMessage(CUIWindow* pWnd, s16 msg, void* pData)
{
	if (msg == WINDOW_MOUSE_CAPTURE_LOST)
	{
		EndDrag();
		return;
	}
	inherited::SendMessage(pWnd, msg, pData);
}
