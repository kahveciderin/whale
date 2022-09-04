#include "print.hpp"
#include "ast.hpp"
#include <ostream>
std::string indent(int level) {
    std::string s;
    for (int i = 0; i < level; ++i) {
        s += "  ";
    }
    return s;
}

void ASTBaseType::print(std::ostream &out, int level) const {
  out << indent(level) << "Type: " << name_ << std::endl;
}

void ASTNodeList::print(std::ostream &out, int level) const {
  out << indent(level) << "NodeList:\n";
  for (auto node : nodes_) {
    node->print(out, level + 1);
  }
}

void ASTArrayLiteral::print(std::ostream &out, int level) const {
  out << indent(level) << "ArrayLiteral: " << std::endl;
  list_->print(out, level + 1);
}

void ASTTemplate::print(std::ostream &out, int level) const {
  out << indent(level) << "Template: " << name_ << std::endl;
  type_->print(out, level + 1);
}

void ASTFunctionArg::print(std::ostream &out, int level) const {
  out << indent(level) << "Argument: " << name_ << std::endl;
  type_->print(out, level + 1);
}

void ASTLambda::print(std::ostream &out, int level) const {
  out << indent(level) << "Lambda: " << std::endl;
  for (auto arg : args_) {
    arg->print(out, level + 1);
  }
  body_->print(out, level + 1);
  type_->print(out, level + 1);
}

void ASTNativeFunction::print(std::ostream &out, int level) const {
  out << indent(level) << "native @ " << function_;
}

void ASTPointer::print(std::ostream &out, int level) const {
  out << indent(level) << "Pointer: " << std::endl;
  type_->print(out, level + 1);
}

void ASTArray::print(std::ostream &out, int level) const {
  out << indent(level) << "Array: " << std::endl;
  type_->print(out, level + 1);
  size_->print(out, level + 1);
}

void ASTNumber::print(std::ostream &out, int level) const {
  out << indent(level) << "Number: " << value_ << std::endl;
}

void ASTChar::print(std::ostream &out, int level) const {
  out << indent(level) << "Char: " << value_ << std::endl;
}

void ASTDouble::print(std::ostream &out, int level) const {
  out << indent(level) << "Double: " << value_ << std::endl;
}

void ASTString::print(std::ostream &out, int level) const {
  out << indent(level) << "String: \"" << value_ << "\"" << std::endl;
}

void ASTBinaryOp::print(std::ostream &out, int level) const {
  out << indent(level) << "BinaryOp: " << op_ << std::endl;
  left_->print(out, level + 1);
  right_->print(out, level + 1);
}

void ASTCast::print(std::ostream &out, int level) const {
  out << indent(level) << "Cast: " << std::endl;
  type_->print(out, level + 1);
  value_->print(out, level + 1);
}

void ASTVariableDecl::print(std::ostream &out, int level) const {
  out << indent(level) << "VariableDecl: " << name_ << std::endl;
  type_->print(out, level + 1);
  if (value_ != nullptr) {
    value_->print(out, level + 1);
  }
}

void ASTVariable::print(std::ostream &out, int level) const {
  out << indent(level) << "Variable: " << name_ << std::endl;
}

void ASTVariableAssign::print(std::ostream &out, int level) const {
  out << indent(level) << "VariableAssign: " << name_ << std::endl;
  value_->print(out, level + 1);
}

void ASTFunctionCall::print(std::ostream &out, int level) const {
  out << indent(level) << "FunctionCall: " << std::endl;
  pointer_->print(out, level + 1);
  args_->print(out, level + 1);
}

void ASTIf::print(std::ostream &out, int level) const {
  out << indent(level) << "If: " << std::endl;
  condition_->print(out, level + 1);
  body_->print(out, level + 1);
  if (elseBody_ != nullptr) {
    elseBody_->print(out, level + 1);
  }
}

void ASTWhile::print(std::ostream &out, int level) const {
  out << indent(level) << "While: " << std::endl;
  condition_->print(out, level + 1);
  body_->print(out, level + 1);
}

void ASTReturn::print(std::ostream &out, int level) const {
  out << indent(level) << "Return: " << std::endl;
  if (value_ != nullptr) {
    value_->print(out, level + 1);
  }
}

void ASTRef::print(std::ostream &out, int level) const {
  out << indent(level) << "Ref: " << variable_ << std::endl;
}

void ASTDeref::print(std::ostream &out, int level) const {
  out << indent(level) << "Deref: " << std::endl;
  value_->print(out, level + 1);
}

void ASTNull::print(std::ostream &out, int level) const {
  out << indent(level) << "Null" << std::endl;
}

void ASTVoid::print(std::ostream &out, int level) const {
  out << indent(level) << "Void" << std::endl;
}

void ASTBool::print(std::ostream &out, int level) const {
  out << indent(level) << "Bool: " << value_ << std::endl;
}

void ASTArrayAccess::print(std::ostream &out, int level) const {
  out << indent(level) << "ArrayAccess: " << std::endl;
  array_->print(out, level + 1);
  index_->print(out, level + 1);
}

void ASTForIn::print(std::ostream &out, int level) const {
  out << indent(level) << "ForIn: " << variable_ << std::endl;
  array_->print(out, level + 1);
  body_->print(out, level + 1);
}

void ASTArrayComprehension::print(std::ostream &out, int level) const {
  out << indent(level) << "ArrayComprehension: " << variable_
      << std::endl;
  array_->print(out, level + 1);
  body_->print(out, level + 1);
}

void ASTRange::print(std::ostream &out, int level) const {
  out << indent(level) << "Range: " << std::endl;
  start_->print(out, level + 1);
  end_->print(out, level + 1);
}

void ASTFunctionType::print(std::ostream &out, int level) const {
  out << indent(level) << "FunctionType: " << std::endl;
  out << indent(level + 1) << "Return Type: " << std::endl;
  ret_type_->print(out, level + 1);
  out << indent(level + 1) << "Arg Types: " << std::endl;
  for (auto arg : arg_types_) {
    arg->print(out, level + 1);
  }
};