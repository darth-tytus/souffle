/*
 * Souffle - A Datalog Compiler
 * Copyright (c) 2013, 2014, Oracle and/or its affiliates. All rights reserved
 * Licensed under the Universal Permissive License v 1.0 as shown at:
 * - https://opensource.org/licenses/UPL
 * - <souffle root>/licenses/SOUFFLE-UPL.txt
 */

/************************************************************************
 *
 * @file AstTypeDeclaration.h
 *
 * Defines a type, i.e., disjoint supersets of the universe
 *
 ***********************************************************************/

#pragma once

#include "RamTypes.h"
#include "SrcLocation.h"
#include "ast/AstAttribute.h"
#include "ast/AstNode.h"
#include "ast/AstQualifiedName.h"
#include "utility/ContainerUtil.h"
#include "utility/MiscUtil.h"
#include "utility/StreamUtil.h"
#include "utility/tinyformat.h"
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace souffle {

/**
 *  @class TypeDeclaration
 *  @brief An abstract base class for types within the AST.
 */
class AstTypeDeclaration : public AstNode {
public:
    AstTypeDeclaration(AstQualifiedName name = {}, SrcLocation loc = {})
            : AstNode(std::move(loc)), name(std::move(name)) {}

    /** get type name */
    const AstQualifiedName& getQualifiedName() const {
        return name;
    }

    /** set type name */
    void setQualifiedName(AstQualifiedName name) {
        this->name = std::move(name);
    }

    AstTypeDeclaration* clone() const override = 0;

private:
    /** type name */
    AstQualifiedName name;
};

/**
 * A subset type. Can be derived from any type except union.
 */
class AstSubsetTypeDeclaration : public AstTypeDeclaration {
public:
    AstSubsetTypeDeclaration(AstQualifiedName name, AstQualifiedName baseTypeName, SrcLocation loc = {})
            : AstTypeDeclaration(std::move(name), std::move(loc)), baseType(std::move(baseTypeName)) {}

    AstSubsetTypeDeclaration* clone() const override {
        return new AstSubsetTypeDeclaration(getQualifiedName(), getBaseType(), getSrcLoc());
    }

    const AstQualifiedName& getBaseType() const {
        return baseType;
    }

protected:
    void print(std::ostream& os) const override {
        os << tfm::format(".type %s <: %s", getQualifiedName(), getBaseType());
    }

    bool equal(const AstNode& node) const override {
        const auto& other = static_cast<const AstSubsetTypeDeclaration&>(node);
        return getQualifiedName() == other.getQualifiedName() && baseType == other.baseType;
    }

private:
    const AstQualifiedName baseType;
};

/**
 * A union type combines multiple types into a new super type.
 * Each of the enumerated types become a sub-type of the new
 * union type.
 */
class AstUnionTypeDeclaration : public AstTypeDeclaration {
public:
    AstUnionTypeDeclaration(AstQualifiedName name, std::vector<AstQualifiedName> types, SrcLocation loc = {})
            : AstTypeDeclaration(std::move(name), std::move(loc)), types(std::move(types)) {}

    /** Obtains a reference to the list element types */
    const std::vector<AstQualifiedName>& getTypes() const {
        return types;
    }

    /** Adds another element type */
    void add(AstQualifiedName type) {
        types.push_back(std::move(type));
    }

    /** Set variant type */
    void setVariantType(size_t idx, AstQualifiedName type) {
        types.at(idx) = std::move(type);
    }

    AstUnionTypeDeclaration* clone() const override {
        return new AstUnionTypeDeclaration(getQualifiedName(), types, getSrcLoc());
    }

protected:
    void print(std::ostream& os) const override {
        os << tfm::format(".type %s = %s", getQualifiedName(), join(types, " | "));
    }

    bool equal(const AstNode& node) const override {
        const auto& other = static_cast<const AstUnionTypeDeclaration&>(node);
        return getQualifiedName() == other.getQualifiedName() && types == other.types;
    }

private:
    /** The list of types aggregated by this union type */
    std::vector<AstQualifiedName> types;
};

/**
 * A record type aggregates a list of fields into a new type.
 * Each record type has a name making it unique. Two record
 * types are unrelated to all other types (they do not have
 * any super or sub types).
 */
class AstRecordTypeDeclaration : public AstTypeDeclaration {
public:
    AstRecordTypeDeclaration(AstQualifiedName name, VecOwn<AstAttribute> fields, SrcLocation loc = {})
            : AstTypeDeclaration(std::move(name), std::move(loc)), fields(std::move(fields)) {}

    /** add field to record type */
    void add(std::string name, AstQualifiedName type) {
        fields.push_back(mk<AstAttribute>(std::move(name), std::move(type)));
    }

    /** get fields of record */
    std::vector<AstAttribute*> getFields() const {
        return toPtrVector(fields);
    }

    /** set field type */
    void setFieldType(size_t idx, AstQualifiedName type) {
        fields.at(idx)->setTypeName(std::move(type));
    }

    AstRecordTypeDeclaration* clone() const override {
        return new AstRecordTypeDeclaration(getQualifiedName(), souffle::clone(fields), getSrcLoc());
    }

protected:
    void print(std::ostream& os) const override {
        os << tfm::format(".type %s = [%s]", getQualifiedName(), join(fields, ", "));
    }

    bool equal(const AstNode& node) const override {
        const auto& other = dynamic_cast<const AstRecordTypeDeclaration&>(node);
        return getQualifiedName() == other.getQualifiedName() && equal_targets(fields, other.fields);
    }

private:
    /** record fields */
    VecOwn<AstAttribute> fields;
};

}  // end of namespace souffle
