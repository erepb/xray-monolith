#include "pch_script.h"
#include "UIRangeBar.h"

using namespace luabind;

#pragma optimize("s",on)

void CUIRangeBar::script_register(lua_State* L)
{
	module(L)
	[
		class_<CUIRangeBar, CUIWindow>("CUIRangeBar")
		.def(constructor<>())
		.def("SetBounds", &CUIRangeBar::SetBounds)
		.def("SetStep", &CUIRangeBar::SetStep)
		.def("GetCount", &CUIRangeBar::GetCount)
		.def("SetValue", &CUIRangeBar::SetValue)
		.def("GetValue", &CUIRangeBar::GetValue)
		.def("SetValues", &CUIRangeBar::SetValues)
		.def("GetMinValue", &CUIRangeBar::GetMinValue)
		.def("GetMaxValue", &CUIRangeBar::GetMaxValue)
		.def("SetFillColor", &CUIRangeBar::SetFillColor)
	];
}
