/**
 * WISE_Scenario_Growth_Module: ScenarioExportRules.cpp
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

#include "angles.h"
#include "ScenarioExportRules.h"
#include "FireEngine_ext.h"
#include <boost/algorithm/string.hpp>


ExportRule::ExportRule() {
	operation = 0;
	ignition = nullptr;
	asset = nullptr;
	assetIndex = (ULONG)-1;
}


ExportRule::ExportRule(const ExportRule& rule) {
	operation = rule.operation;
	ignition = rule.ignition;
	asset = rule.asset;
	assetIndex = rule.assetIndex;
	name = rule.name;
	value = rule.value;
}


ScenarioExportRules::ScenarioExportRules(const ScenarioExportRules& rules) {
	ExportRule* r = rules.m_rules.LH_Head();
	while (r->LN_Succ()) {
		ExportRule* cr = new ExportRule(*r);
		if (cr)
			m_rules.AddTail(cr);
		r = r->LN_Succ();
	}
}


ScenarioExportRules::~ScenarioExportRules() {
	ClearAllRules();
}


void ScenarioExportRules::ClearAllRules() {
	ExportRule *r;
	while (r = m_rules.RemHead())
		delete r;
}


void ScenarioExportRules::AddAttributeString(CCWFGM_Ignition *ignition, const TCHAR *attribute, const TCHAR *value) {
	ExportRule *r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY;
	r->ignition = ignition;
	r->name = attribute;
	r->value = std::string(value);
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeString(CCWFGM_Asset* asset, ULONG assetIndex, const TCHAR* attribute, const TCHAR* value) {
	ExportRule* r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY;
	r->ignition = nullptr;
	r->asset = asset;
	r->assetIndex = assetIndex;
	r->name = attribute;
	r->value = std::string(value);
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeDouble(CCWFGM_Ignition* ignition, const TCHAR* attribute, const double value) {
	ExportRule* r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_DOUBLEPROPERTY;
	r->ignition = ignition;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeDouble(CCWFGM_Asset* asset, ULONG assetIndex, const TCHAR* attribute, const double value) {
	ExportRule* r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY;
	r->ignition = nullptr;
	r->asset = asset;
	r->assetIndex = assetIndex;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeInt32(CCWFGM_Ignition* ignition, const TCHAR* attribute, const std::int32_t value) {
	ExportRule* r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_INT32PROPERTY;
	r->ignition = ignition;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeInt32(CCWFGM_Asset* asset, ULONG assetIndex, const TCHAR* attribute, const std::int32_t value) {
	ExportRule* r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY;
	r->ignition = nullptr;
	r->asset = asset;
	r->assetIndex = assetIndex;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeInt64(CCWFGM_Ignition* ignition, const TCHAR* attribute, const std::int64_t value) {
	ExportRule* r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_INT64PROPERTY;
	r->ignition = ignition;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeInt64(CCWFGM_Asset* asset, ULONG assetIndex, const TCHAR* attribute, const std::int64_t value) {
	ExportRule* r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY;
	r->ignition = nullptr;
	r->asset = asset;
	r->assetIndex = assetIndex;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeString(const TCHAR *attribute, const TCHAR *value) {
	ExportRule *r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY;
	r->ignition = nullptr;
	r->name = attribute;
	r->value = std::string(value);
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeDouble(const TCHAR* attribute, const double value) {
	ExportRule* r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_DOUBLEPROPERTY;
	r->ignition = nullptr;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeInt32(const TCHAR* attribute, const std::int32_t value) {
	ExportRule* r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_INT32PROPERTY;
	r->ignition = nullptr;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddAttributeInt64(const TCHAR* attribute, const std::int64_t value) {
	ExportRule* r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_INT64PROPERTY;
	r->ignition = nullptr;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddStatistic(CCWFGM_Ignition *ignition, const TCHAR *attribute, std::uint16_t stat, UnitConvert::STORAGE_UNIT units) {
	std::uint64_t value = ((std::uint64_t)units) << 32 | stat;
	ExportRule *r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_STATPROPERTY;
	r->ignition = ignition;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::AddStatistic(const TCHAR *attribute, std::uint16_t stat, UnitConvert::STORAGE_UNIT units) {
	std::uint64_t value = ((std::uint64_t)units) << 32 | stat;
	ExportRule *r = new ExportRule();
	r->operation = CWFGM_SCENARIO_OPTION_EXPORTRULE_STATPROPERTY;
	r->ignition = NULL;
	r->name = attribute;
	r->value = value;
	m_rules.AddTail(r);
}


void ScenarioExportRules::addOperation(bool add, std::uint32_t operation, std::uint64_t byref) {
	if (add) {
		ExportRule* r = m_rules.LH_Head();
		while (r->LN_Succ()) {
			if (r->operation == operation)
				break;
			r = r->LN_Succ();
		}
		if (!r->LN_Succ()) {
			r = new ExportRule();
			m_rules.AddTail(r);
		}
		r->operation = operation;
		r->ignition = nullptr;
		r->value = byref;
	}
	else {
		ExportRule* r = m_rules.LH_Head();
		while (r->LN_Succ()) {
			if (r->operation == operation)
				break;
			r = r->LN_Succ();
		}
		if (r->LN_Succ()) {
			m_rules.Remove(r);
			delete r;
		}
	}
}


void ScenarioExportRules::AddOperation(bool append_set, bool export_set, ULONGLONG byref) {
	addOperation(append_set, CWFGM_SCENARIO_OPTION_EXPORTRULE_OPERATION_APPEND, byref);
	addOperation(export_set, CWFGM_SCENARIO_OPTION_EXPORTRULE_OPERATION_EXPORT, byref);
}


std::uint64_t ScenarioExportRules::GetAppendOperation() const {
	ExportRule* r = m_rules.LH_Head();
	while (r->LN_Succ()) {
		if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_OPERATION_APPEND) {
			std::uint64_t retval;
			VariantToUInt64_(r->value, &retval);
			return retval;
		}
		r = r->LN_Succ();
	}
	return 0;
}


std::uint64_t ScenarioExportRules::GetExportOperation() const {
	ExportRule* r = m_rules.LH_Head();
	while (r->LN_Succ()) {
		if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_OPERATION_EXPORT) {
			std::uint64_t retval;
			VariantToUInt64_(r->value, &retval);
			return retval;
		}
		r = r->LN_Succ();
	}
	return 0;
}


ExportRule *ScenarioExportRules::GetNextAttributeName(std::uint32_t &counter) const {
	std::vector<std::string> names;
	ExportRule *r = m_rules.LH_Head();
	while (r->LN_Succ()) {
		if ((r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_STATPROPERTY) ||
			(r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY) ||
			(r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_DOUBLEPROPERTY) ||
			(r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_INT32PROPERTY) ||
			(r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_INT64PROPERTY)) {
			if ((std::uint32_t)names.size() < counter) {
				int i;
				for (i = 0; i < names.size(); i++)
					if (boost::iequals(names[i], r->name))
						break;
				if (i == (int)names.size())
					names.push_back(r->name);
			} else if (names.size() == counter) {
				int i;
				for (i = 0; i < names.size(); i++) {
					if (boost::iequals(names[i], r->name))
						break;
				}
				if (i == names.size()) {
					counter++;
					return r;
				}
				
			}
		}
		r = r->LN_Succ();
	}
	return NULL;
}


std::uint32_t ScenarioExportRules::GetTextAttributeMaxLength(const TCHAR *attribute) const {
	size_t max_value_length = 0;
	ExportRule *r = m_rules.LH_Head();
	while (r->LN_Succ()) {
		if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY) {
			if (boost::iequals(attribute, r->name)) {
				std::string rvalue;
				bool failed = false;
				try { rvalue = std::get<std::string>(r->value); } catch (std::bad_variant_access &) { weak_assert(false); failed = true; };

				if (!failed) {
					size_t len = rvalue.length();
					if (len > max_value_length)
						max_value_length = len;
				}
			}
		}
		r = r->LN_Succ();
	}
	return max_value_length;
}


ExportRule *ScenarioExportRules::GetNextTextAttributeName(std::uint32_t &counter) const {
	std::vector<std::string> names;
	ExportRule *r = m_rules.LH_Head();
	while (r->LN_Succ()) {
		if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_TEXTPROPERTY) {
			if (names.size() < counter) {
				int i;
				for (i = 0; i < names.size(); i++)
					if (boost::iequals(names[i], r->name))
						break;
				if (i == names.size())
					names.push_back(r->name);
			} else if (names.size() == counter) {
				int i;
				for (i = 0; i < names.size(); i++) {
					if (boost::iequals(names[i], r->name))
						break;
				}
				if (i == names.size()) {
					std::string rvalue;
					bool failed = false;
					try { rvalue = std::get<std::string>(r->value); } catch (std::bad_variant_access &) { weak_assert(false); failed = true; };
					
					size_t max_value_length;
					if (!failed)
						max_value_length = rvalue.length();
					else
						max_value_length = 0;

					if (!failed) {
						std::string cname(r->name);

						ExportRule *rr = r;
						while (r->LN_Succ()) {
							std::string rname(r->name);
							if (boost::iequals(rname, cname)) {
								std::string rvalue2;
								bool failed = false;
								try { rvalue2 = std::get<std::string>(r->value); }
								catch (std::bad_variant_access &) { weak_assert(false); failed = true; };

								if (!failed) {
									size_t len = rvalue2.length();
									if (len > max_value_length) {
										rr = r;
										max_value_length = len;
									}
								}
							}
							r = r->LN_Succ();
						}
						counter++;
						return rr;
					}
				}
				
			}
		}
		r = r->LN_Succ();
	}
	return NULL;
}


ExportRule *ScenarioExportRules::GetNextStatisticName(std::uint32_t &counter) const {
	std::vector<std::string> names;
	ExportRule *r = m_rules.LH_Head();
	while (r->LN_Succ()) {
		if (r->operation == CWFGM_SCENARIO_OPTION_EXPORTRULE_STATPROPERTY) {
			if (names.size() < counter) {
				int i;
				for (i = 0; i < names.size(); i++)
					if (boost::iequals(names[i], r->name))
						break;
				if (i == names.size())
					names.push_back(r->name);
			} else if (names.size() == counter) {
				int i;
				for (i = 0; i < names.size(); i++) {
					if (boost::iequals(names[i], r->name))
						break;
				}
				if (i == names.size()) {
					counter++;
					return r;
				}
				
			}
		}
		r = r->LN_Succ();
	}
	return nullptr;
}


ExportRule* ScenarioExportRules::FindAttributeName(const TCHAR* attribute) const {
	ExportRule* er = m_rules.LH_Head();
	while (er->LN_Succ()) {
		std::string name(attribute);
		if (!boost::iequals(name, er->name))
			return er;
		er = er->LN_Succ();
	}
	return NULL;
}
