#pragma once
#include "UITabButton.h"

class CUIStatic;
class CUIWindow;

class CUIScrollArrowButton : public CUITabButton
{
	typedef CUITabButton inherited;
public:
	virtual bool OnMouseDown(int mouse_btn);
	virtual bool OnMouseAction(float x, float y, EUIMessages mouse_action);
};

// LEGACY static-strip arrow
class CUIScrollArrowHalvesButton : public CUIScrollArrowButton
{
	typedef CUIScrollArrowButton inherited;
public:
	CUIScrollArrowHalvesButton();

	void SetupHalves(const shared_str& art_base);
	void LayoutHalves();

	virtual void Update();

protected:
	int  CurrentIBState();
	void ApplyHalfArt(int ib_state);

	CUIStatic* m_half[2];
	shared_str m_half_base;
	int        m_applied_state;
};

class CUITabScrollArrows
{
public:
	enum { eNone = -1, eLeft = 0, eRight = 1 };

	CUITabScrollArrows();
	~CUITabScrollArrows();

	void Init(CUIWindow* parent, CUIWindow* msg_target);
	void SetDeclaredArt(LPCSTR base);

	void  EnsureBuilt(CUITabButton* ref);
	void  Layout(float view_right, float strip_y);
	void  Show(bool visible);
	void  SetEnabled(int side, bool enabled);
	void  Draw();
	void  ApplyHitClips(const Fvector2& origin);
	int   SideOf(const CUIWindow* clicked) const;
	float Width(int side) const { return m_arrow[side] ? m_arrow[side]->GetWndSize().x : 0.0f; }

private:
	enum EStrategy { eCapHalves, eFrameline };

	void Build(CUITabButton* ref, EStrategy strat);

	CUIWindow* m_parent;
	CUIWindow* m_msg_target;
	shared_str m_declared_art;

	CUIScrollArrowButton* m_arrow[2];
};
