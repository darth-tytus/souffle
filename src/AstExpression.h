/*
 * Souffle - A Datalog Compiler
 * Copyright (c) 2020, The Souffle Developers. All rights reserved.
 * Licensed under the Universal Permissive License v 1.0 as shown at:
 * - https://opensource.org/licenses/UPL
 * - <souffle root>/licenses/SOUFFLE-UPL.txt
 */

/************************************************************************
 *
 * @file AstExpression.h
 *
 * Define classes for AST expressions involving literals.
 *
 ***********************************************************************/

#pragma once

#include "AstAbstract.h"
#include "utility/ContainerUtil.h"
#include "utility/MiscUtil.h"
#include "utility/StreamUtil.h"
#include "utility/tinyformat.h"

namespace souffle {

class AstDisjunction : public AstExpression {
public:
    AstDisjunction(VecOwn<AstExpression> args, SrcLocation loc = {})
            : AstExpression(std::move(loc)), arguments(std::move(args)) {}

    std::vector<AstExpression*> getArguments() const {
        return toPtrVector(arguments);
    }

    AstDisjunction* clone() const override {
        return new AstDisjunction(souffle::clone(arguments), getSrcLoc());
    }

    void apply(const AstNodeMapper& map) override {
        for (auto& arg : arguments) {
            arg = map(std::move(arg));
        }
    }

    std::vector<const AstNode*> getChildNodes() const override {
        std::vector<const AstNode*> children;
        for (auto& cur : arguments) {
            children.push_back(cur.get());
        }
        return children;
    }

protected:
    void print(std::ostream& os) const override {
        os << join(arguments, ";\n");
    }

    bool equal(const AstNode& node) const override {
        const auto& other = dynamic_cast<const AstDisjunction&>(node);
        return equal_targets(arguments, other.arguments);
    }

private:
    VecOwn<AstExpression> arguments;
};

class AstConjunction : public AstExpression {
public:
    AstConjunction(VecOwn<AstExpression> args, SrcLocation loc = {})
            : AstExpression(std::move(loc)), arguments(std::move(args)) {}

    std::vector<AstExpression*> getArguments() const {
        return toPtrVector(arguments);
    }

    AstConjunction* clone() const override {
        return new AstConjunction(souffle::clone(arguments), getSrcLoc());
    }

    void apply(const AstNodeMapper& map) override {
        for (auto& arg : arguments) {
            arg = map(std::move(arg));
        }
    }

    std::vector<const AstNode*> getChildNodes() const override {
        std::vector<const AstNode*> children;
        for (auto& cur : arguments) {
            children.push_back(cur.get());
        }
        return children;
    }

protected:
    void print(std::ostream& os) const override {
        os << join(arguments, ", ");
    }

    bool equal(const AstNode& node) const override {
        const auto& other = dynamic_cast<const AstConjunction&>(node);
        return equal_targets(arguments, other.arguments);
    }

private:
    VecOwn<AstExpression> arguments;
};

class AstExprNegation : public AstExpression {
    AstExprNegation(Own<AstExpression> arg, SrcLocation loc = {})
            : AstExpression(std::move(loc)), argument(std::move(arg)) {
        assert(argument != nullptr);
    }

    AstExpression* getArgument() const {
        return argument.get();
    }

    AstExprNegation* clone() const override {
        return new AstExprNegation(souffle::clone(argument), getSrcLoc());
    }

    void apply(const AstNodeMapper& map) override {
        argument = map(std::move(argument));
    }

    std::vector<const AstNode*> getChildNodes() const override {
        return {argument.get()};
    }

protected:
    void print(std::ostream& os) const override {
        os << tfm::format("!(%s)", *argument);
    }

    bool equal(const AstNode& node) const override {
        const auto& other = dynamic_cast<const AstExprNegation&>(node);
        return equal_ptr(argument, other.argument);
    }

private:
    Own<AstExpression> argument;
};

}  // namespace souffle
