/**
 * WISE_Scenario_Growth_Module: ScenarioExportRules.h
 * Copyright (C) 2023  WISE
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#if !defined(AFX_SCENARIOEXPORTRULES_H__10557D15_268E_11D4_BCD9_00A0833B1640__INCLUDED_)
#define AFX_SCENARIOEXPORTRULES_H__10557D15_268E_11D4_BCD9_00A0833B1640__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "FireEngine.h"
#include "linklist.h"
#include "convert.h"
#include "SExportRule.h"



#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(push, 8)
#endif

class FIRECOM_API ExportRule : public MinNode, public SExportRule {
    public:
	ExportRule();
	ExportRule(const ExportRule& rule);
	~ExportRule() = default;

	ExportRule *LN_Succ() const			{ return (ExportRule *)MinNode::LN_Succ(); };
	ExportRule *LN_Pred() const			{ return (ExportRule *)MinNode::LN_Pred(); };

	DECLARE_OBJECT_CACHE_MT(ExportRule, ExportRule)
};


class FIRECOM_API ScenarioExportRules {
    public:
	ScenarioExportRules() = default;
	ScenarioExportRules(const ScenarioExportRules& rules);
	~ScenarioExportRules();

	MinListTempl<ExportRule> m_rules;

	void ClearAllRules();

	void AddAttributeString(CCWFGM_Ignition* ignition, const TCHAR* attribute, const TCHAR* value);
	void AddAttributeDouble(CCWFGM_Ignition* ignition, const TCHAR* attribute, const double value);
	void AddAttributeInt32(CCWFGM_Ignition* ignition, const TCHAR* attribute, const std::int32_t value);
	void AddAttributeInt64(CCWFGM_Ignition* ignition, const TCHAR* attribute, const std::int64_t value);

	void AddAttributeString(CCWFGM_Asset* asset, ULONG assetIndex, const TCHAR* attribute, const TCHAR* value);
	void AddAttributeDouble(CCWFGM_Asset* asset, ULONG assetIndex, const TCHAR* attribute, const double value);
	void AddAttributeInt32(CCWFGM_Asset* asset, ULONG assetIndex, const TCHAR* attribute, const std::int32_t value);
	void AddAttributeInt64(CCWFGM_Asset* asset, ULONG assetIndex, const TCHAR* attribute, const std::int64_t value);

	void AddAttributeString(const TCHAR* attribute, const TCHAR* value);
	void AddAttributeDouble(const TCHAR* attribute, const double value);
	void AddAttributeInt32(const TCHAR* attribute, const std::int32_t value);
	void AddAttributeInt64(const TCHAR* attribute, const std::int64_t value);
	void AddStatistic(CCWFGM_Ignition *ignition, const TCHAR *attribute, std::uint16_t stat, UnitConvert::STORAGE_UNIT units);
	void AddStatistic(const TCHAR *attribute, std::uint16_t stat, UnitConvert::STORAGE_UNIT units);
	void AddOperation(bool append_set, bool export_set, ULONGLONG byref);

	ExportRule *GetNextAttributeName(std::uint32_t &counter) const;
	ExportRule *GetNextTextAttributeName(std::uint32_t &counter) const;
	ExportRule *GetNextStatisticName(std::uint32_t &counter) const;
	ExportRule* FindAttributeName(const TCHAR* attribute) const;

	ULONGLONG GetAppendOperation() const;
	ULONGLONG GetExportOperation() const;

	std::uint32_t GetTextAttributeMaxLength(const TCHAR *attribute) const;

private:
	void addOperation(bool add, std::uint32_t operation, std::uint64_t byref);
};

#ifdef HSS_SHOULD_PRAGMA_PACK
#pragma pack(pop)
#endif

#endif // !defined(AFX_SCENARIOIGNITION_H__10557D15_268E_11D4_BCD9_00A0833B1640__INCLUDED_)
