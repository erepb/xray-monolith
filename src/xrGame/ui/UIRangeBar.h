#pragma once

#include "UI_IB_Static.h"
#include "../../xrServerEntities/script_export_space.h"

class CUIStatic;

class CUIRangeBar : public CUI_IB_FrameLineWnd
{
	typedef CUI_IB_FrameLineWnd inherited;
public:
	CUIRangeBar();
	virtual ~CUIRangeBar();

	void InitRangeBar(Fvector2 pos, Fvector2 size, int count, LPCSTR track, LPCSTR knob,
	                  LPCSTR knob_min, LPCSTR knob_max);
	void SetFillArt(LPCSTR base, float height);
	void SetFillColor(u32 argb, float height);

	void  SetBounds(float bmin, float bmax);
	void  SetStep(float step);
	int   GetCount() const { return (int)m_val.size(); }
	void  SetValue(int rank, float v);
	float GetValue(int rank) const;
	void  SetValues(float a, float b);
	float GetMinValue() const { return GetValue(0); }
	float GetMaxValue() const { return GetValue(GetCount() - 1); }

	virtual void Enable(bool status);
	virtual void Draw();
	virtual void Update();
	virtual bool OnMouseAction(float x, float y, EUIMessages mouse_action);
	virtual void SendMessage(CUIWindow* pWnd, s16 msg, void* pData = NULL);

protected:
	float knob_w() const;
	float travel() const;
	float span() const;
	float ValueToX(float v) const;
	float XToValue(float px) const;
	float Quantize(float v) const;
	float ClampToNeighbours(int rank, float v) const;
	void  Requantize();
	float KnobCenterX(int rank) const;

	int  HitKnob(float x) const;
	int  NearestKnob(float x) const;
	void StackOf(int rank, int& lo, int& hi) const;
	void MoveKnob(int rank, float px);
	void EndDrag();
	void Layout();
	void UpdateKnobStates();

private:
	void SetFill(CUIWindow* f, float height);

protected:
	xr_vector<CUI_IB_Static*> m_knob;
	xr_vector<float> m_val; // non-decreasing
	CUIWindow* m_fill;

	float m_bmin;
	float m_bmax;
	float m_step;

	int m_drag; // -1 = none
	int m_pending; // -1 = none, else the knob pressed on a stack; pick deferred to the first move
	int m_top; // drawn last, hit first
	float m_grab_dx;

DECLARE_SCRIPT_REGISTER_FUNCTION
};
