#include "ast.hpp"
#include "parser.hpp"
#include <exception>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/IntrinsicInst.h>
#include <llvm/IR/Type.h>
#include <sys/mman.h>

WithPos::WithPos(unsigned long p) : pos_(p) {}

CompilerStackFrame::CompilerStackFrame(CompilerStackFrame *parent) : parent(parent) {}

CompilerStackFrame::~CompilerStackFrame() {
  for (auto thing = this->data.begin(); thing != this->data.end(); thing++) {
    //TODO: This somehow breaks everything. Find a way to add lifetimes properly.
    //TheBuilder->CreateLifetimeEnd(thing->second);
  }
}

void CompilerStackFrame::set(std::string name, llvm::Value *varspace) {
    data[name] = varspace;
}
llvm::Value *CompilerStackFrame::resolve(std::string name) {
    if (data.count(name)) {
        return data[name];
    }
    if (parent) {
    //tail call optimization
    return parent->resolve(name);
    } else {
      auto glob_val = Module->getNamedGlobal(name);
      if (glob_val) {
        return glob_val;
      }
    }
    throw std::string("Undefined variable ") + name;
}


LambdaStackFrame::LambdaStackFrame(CompilerStackFrame *upper, std::function<llvm::Value *(std::string, llvm::Value *)> register_implicit) : parent(upper), implicit_handler(register_implicit) {};
llvm::Value *LambdaStackFrame::resolve(std::string name) {
  if (this->data.count(name)) {
    return data[name];
  }
  if (parent) {
    auto res = parent->resolve(name);
    auto registered = implicit_handler(name, res);
    this->set(name, registered);
    return registered;
  }
  throw "Undefined variable " + name;
}

class ASTNodeList;
class Runner;
class RunnerStackFrame;

ASTNode::ASTNode(unsigned long pos) : WithPos(pos) {}
ASTNode::~ASTNode() {}
ASTType::ASTType(unsigned long pos) : WithPos(pos) {}
ASTType::~ASTType() {}

ASTBaseType::ASTBaseType(const std::string &name, unsigned long pos)
    : ASTType(pos), name_(name) {}


llvm::Type *ASTBaseType::into_llvm_type() {
  ValueType type = parseType(name_);
  switch (type) {
    case ValueType::i32:
      return llvm::Type::getInt32Ty(*TheContext);
    case ValueType::i64:
      return llvm::Type::getInt64Ty(*TheContext);
    case ValueType::fun:
      //opaque pointer
      return llvm::Type::getVoidTy(*TheContext)->getPointerTo();
    case ValueType::float_:
      return llvm::Type::getFloatTy(*TheContext);
    case ValueType::double_:
      return llvm::Type::getDoubleTy(*TheContext);
    case ValueType::void_:
      return llvm::Type::getVoidTy(*TheContext);
    case ValueType::bool_:
      return llvm::Type::getInt1Ty(*TheContext);
    case ValueType::char_:
      return llvm::Type::getInt8Ty(*TheContext);
    case ValueType::pointer:
      return llvm::Type::getPrimitiveType(*TheContext, llvm::Type::PointerTyID);
    default:
      throw std::runtime_error("Unknown type: " + name_);
  }
}

ASTBaseType::~ASTBaseType() {}

ASTNodeList::ASTNodeList(const std::vector<ASTNode *> &nodes, unsigned long pos)
    : ASTNode(pos), nodes_(nodes) {}
ASTNodeList::ASTNodeList(unsigned long pos) : ASTNode(pos) {}

ASTNodeList::~ASTNodeList() {
  for (auto node : nodes_) {
    delete node;
  }
}

void ASTNodeList::add(ASTNode *node) {
  nodes_.push_back(node);
}

llvm::Value *ASTNodeList::codegen(CompilerStackFrame *frame) {
  llvm::Value *retvar = llvm::PoisonValue::get(llvm::Type::getVoidTy(*TheContext));
  for (auto node : nodes_) {
    retvar = node->codegen(frame);
  }
  return retvar;
}
ASTArrayLiteral::ASTArrayLiteral(ASTNodeList *list, unsigned long pos)
    : ASTNode(pos), list_(list) {}

llvm::Value *ASTArrayLiteral::codegen(CompilerStackFrame *frame) {
  std::vector<llvm::Value *> values;
  for (auto node : list_->nodes_) {
    values.push_back(node->codegen(frame));
  }
  auto pointer_to_list = TheBuilder->CreateAlloca(llvm::ArrayType::get(values[0]->getType(), values.size()));
  int idx = 0;
  for (auto value : values) {
    TheBuilder->CreateStore(value, TheBuilder->CreateGEP(value->getType()->getPointerTo(), pointer_to_list, std::vector<llvm::Value *>({TheBuilder->getInt32(0), TheBuilder->getInt32(idx)})));
    idx++;
  }
  return TheBuilder->CreateLoad(pointer_to_list->getAllocatedType(), pointer_to_list);
}

ASTTemplate::ASTTemplate(ASTType *type, const std::string &name, unsigned long pos)
    : ASTType(pos), type_(type), name_(name) {}

ASTTemplate::~ASTTemplate() {};

llvm::Type *ASTTemplate::into_llvm_type() {
  return nullptr;
}


ASTFunctionArg::ASTFunctionArg(ASTType *type, const std::string &name, unsigned long pos)
    : type_(type), name_(name), WithPos(pos) {}


ASTLambda::ASTLambda(std::vector<ASTFunctionArg *> args, ASTNode *body, ASTType *type, unsigned long pos)
    : args_(args), body_(body), type_(type), ASTNode(pos) {}

ASTLambda::~ASTLambda() { delete body_; }

llvm::Value *ASTLambda::codegen(CompilerStackFrame *frame) {
  std::vector<llvm::Type *> implicit_args;
  std::vector<llvm::Value *> replace_later;
  std::vector<llvm::Value *> trampoline_object_members;
  std::vector<llvm::Type *> total_args;
  auto lambda_trampoline_data = llvm::StructType::create(*TheContext);
  total_args.push_back(lambda_trampoline_data->getPointerTo());
  for (auto arg : this->args_) {
    total_args.push_back(arg->type_->into_llvm_type());
  }
  auto lambda_func = llvm::Function::Create(llvm::FunctionType::get(type_->into_llvm_type(), total_args, false), llvm::GlobalValue::InternalLinkage, "", *Module);
  lambda_func->addParamAttr(0, llvm::Attribute::Nest);

  auto lambda_setup_block = llvm::BasicBlock::Create(*TheContext, "", lambda_func);
  
  auto lambda_body = llvm::BasicBlock::Create(*TheContext, "", lambda_func);
  
  auto where_we_left_off = TheBuilder->saveIP();

  
  LambdaStackFrame cur_frame(frame, [&](std::string name, llvm::Value *outer_definition) -> llvm::Value * {
    auto alloc_type = outer_definition->getType();
    //auto proxy_pointer = TheBuilder->CreateGEP(lambda_trampoline_data, lambda_func->getArg(0), std::vector<llvm::Value *>({TheBuilder->getInt32(0), TheBuilder->getInt32(implicit_args.size())}));
    implicit_args.push_back(alloc_type);
    trampoline_object_members.push_back(outer_definition);
    auto the_val = llvm::PoisonValue::get(alloc_type);
    replace_later.push_back(the_val);
    return the_val;
  });
  TheBuilder->SetInsertPoint(lambda_setup_block);
  for (int i=0;i<this->args_.size();i++) {
    auto normal_arg_memory = TheBuilder->CreateAlloca(total_args[i+1]);
    TheBuilder->CreateStore(lambda_func->getArg(i+1), normal_arg_memory);
    cur_frame.set(this->args_[i]->name_, normal_arg_memory);
  }
  
  TheBuilder->SetInsertPoint(lambda_body);
  body_->codegen(&cur_frame);
  lambda_trampoline_data->setBody(implicit_args);

  TheBuilder->SetInsertPoint(lambda_setup_block);
  for (int i=0;i<replace_later.size();i++) {
    replace_later[i]->replaceAllUsesWith(TheBuilder->CreateLoad(implicit_args[i], TheBuilder->CreatePointerCast(TheBuilder->CreateGEP(lambda_trampoline_data, lambda_func->getArg(0), std::vector<llvm::Value *>({TheBuilder->getInt32(0), TheBuilder->getInt32(i)})), implicit_args[i]->getPointerTo())));
  }
  TheBuilder->CreateBr(lambda_body);

  TheBuilder->restoreIP(where_we_left_off);
  
  auto the_trampoline_object = TheBuilder->CreateAlloca(lambda_trampoline_data);
  //load our trampoline
  for (int i = 0; i < trampoline_object_members.size(); i++) {
    TheBuilder->CreateStore(trampoline_object_members[i], TheBuilder->CreateGEP(the_trampoline_object->getAllocatedType(), the_trampoline_object, std::vector<llvm::Value *>({TheBuilder->getInt32(0), TheBuilder->getInt32(i)})));
  }
  //create trampoline intrinsic
  //TODO unmapping this memory sometime is something to be considered.
  auto tramp_memory = TheBuilder->CreateCall(Module->getFunction("mmap"), std::vector<llvm::Value *>({TheBuilder->CreateCast(llvm::Instruction::BitCast, TheBuilder->getInt64(0), TheBuilder->getVoidTy()->getPointerTo()), TheBuilder->getInt64(72), TheBuilder->getInt32(PROT_READ | PROT_WRITE | PROT_EXEC), TheBuilder->getInt32(MAP_ANONYMOUS | MAP_PRIVATE), TheBuilder->getInt32(0), TheBuilder->getInt64(0)}));
  //technically true
  //tramp_memory->setDoesNotAccessMemory();
  tramp_memory->setCannotMerge();
  
  auto init_tramp = TheBuilder->CreateIntrinsic(llvm::Intrinsic::init_trampoline, std::vector<llvm::Type *>({}), std::vector<llvm::Value *>({TheBuilder->CreatePointerCast(tramp_memory, TheBuilder->getInt8PtrTy()), TheBuilder->CreatePointerCast(lambda_func, TheBuilder->getInt8PtrTy()), TheBuilder->CreatePointerCast(the_trampoline_object, TheBuilder->getInt8PtrTy())}));
  init_tramp->setOnlyAccessesArgMemory();
  auto the_final_func_ptr = TheBuilder->CreateIntrinsic(llvm::Intrinsic::adjust_trampoline, std::vector<llvm::Type *>({}), std::vector<llvm::Value *>({TheBuilder->CreatePointerCast(tramp_memory, TheBuilder->getInt8PtrTy())}));
  total_args.erase(total_args.begin());
  
  return TheBuilder->CreateCast(llvm::Instruction::BitCast, the_final_func_ptr, llvm::FunctionType::get(type_->into_llvm_type(), total_args, false)->getPointerTo());
}

ASTNativeFunction::ASTNativeFunction(void (*function)(Runner *, RunnerStackFrame *),
                  unsigned long pos)
    : ASTNode(pos), function_(function) {}

ASTNativeFunction::~ASTNativeFunction() {}



llvm::Value *ASTNativeFunction::codegen(CompilerStackFrame *frame) {
  throw "No compiler-native functions in LLVM compilation allowed!";
}

ASTPointer::ASTPointer(ASTType *type, unsigned long pos)
    : ASTType(pos), type_(type) {}

ASTPointer::~ASTPointer() {}

llvm::Type *ASTPointer::into_llvm_type() {
  return this->type_->into_llvm_type()->getPointerTo();
}


ASTArray::ASTArray(ASTType *type, ASTNode *size, unsigned long pos)
    : ASTType(pos), type_(type), size_(size) {}

ASTArray::~ASTArray() {}

llvm::Type *ASTArray::into_llvm_type() {
  return this->type_->into_llvm_type()->getPointerTo();
}

ASTNumber::ASTNumber(long int value, unsigned long pos)
    : ASTNode(pos), value_(value) {}

llvm::Value *ASTNumber::codegen(CompilerStackFrame *frame) {
  return TheBuilder->getInt64(value_);
}

ASTChar::ASTChar(char value, unsigned long pos)
    : ASTNode(pos), value_(value) {}


llvm::Value *ASTChar::codegen(CompilerStackFrame *frame) {
  return TheBuilder->getInt8(value_);
}


ASTDouble::ASTDouble(double value, unsigned long pos)
    : ASTNode(pos), value_(value) {}

llvm::Value *ASTDouble::codegen(CompilerStackFrame *frame) {
  return llvm::ConstantFP::get(TheBuilder->getDoubleTy(), value_);
}

ASTString::ASTString(const std::string &value, unsigned long pos)
    : ASTNode(pos), value_(value) {}

llvm::Value *ASTString::codegen(CompilerStackFrame *frame) {
  return TheBuilder->CreateGlobalStringPtr(value_);
}

ASTBinaryOp::ASTBinaryOp(std::string op, ASTNode *left, ASTNode *right,
            unsigned long pos)
    : ASTNode(pos), op_(op), left_(left), right_(right) {}

ASTBinaryOp::~ASTBinaryOp() {
  delete left_;
  delete right_;
}

llvm::Value *ASTBinaryOp::codegen(CompilerStackFrame *frame) {
  auto leftval = left_->codegen(frame);
  auto rightval = right_->codegen(frame);
  using BinaryOps = llvm::Instruction::BinaryOps;
  BinaryOps op;
  bool floatop = leftval->getType()->isFloatingPointTy() || rightval->getType()->isFloatingPointTy();
  if (floatop) {
    if (!leftval->getType()->isFloatingPointTy()) {
      leftval = TheBuilder->CreateCast(llvm::Instruction::CastOps::SIToFP, leftval, rightval->getType());
    } else if (!rightval->getType()->isFloatingPointTy()) {
      rightval = TheBuilder->CreateCast(llvm::Instruction::CastOps::SIToFP, rightval, leftval->getType());
    }
  } else {
    if (leftval->getType()->canLosslesslyBitCastTo(rightval->getType())) {
      leftval = TheBuilder->CreateSExtOrBitCast(leftval, rightval->getType());
    } else {
      rightval = TheBuilder->CreateSExtOrBitCast(rightval, leftval->getType());
    }
  }
  
  if (op_ == "+") {
    if (floatop) {
      op = BinaryOps::FAdd;
    } else {
      op = BinaryOps::Add;
    }
  } else if (op_ == "-") {
    if (floatop) {
      op = BinaryOps::FSub;
    } else {
      op = BinaryOps::Sub;
    }
  } else if (op_ == "*") {
    if (floatop) {
      op = BinaryOps::FMul;
    } else {
      op = BinaryOps::Mul;
    }
  } else if (op_ == "/") {
    op = BinaryOps::FDiv;
  } else if (op_ == "//") {
    op = BinaryOps::SDiv;
  } else if (op_ == "%") {
    if (floatop) {
      op = BinaryOps::FRem;
    } else {
      op = BinaryOps::SRem;
    }
  } else if (op_ == "<<") {
    op = BinaryOps::Shl;
  } else if (op_ == ">>") {
    op = BinaryOps::LShr;
  } else if (op_ == "&") {
    op = BinaryOps::And;
  } else if (op_ == "|") {
    op = BinaryOps::Or;
  } else if (op_ == "^") {
    op = BinaryOps::Xor;
  } else if (op_ == "&&") {
    leftval = TheBuilder->CreateIsNotNull(leftval);
    rightval = TheBuilder->CreateIsNotNull(rightval);
    op = BinaryOps::And;
  } else if (op_ == "||") {
    leftval = TheBuilder->CreateIsNotNull(leftval);
    rightval = TheBuilder->CreateIsNotNull(rightval);
    op = BinaryOps::Or;
  } else if (op_ == "==") {
    if (floatop) {
      return TheBuilder->CreateFCmpOEQ(leftval, rightval);
    } else {
      return TheBuilder->CreateICmpEQ(leftval, rightval);
    }
    
  } else if (op_ == "!=") {
    if (floatop) {
      return TheBuilder->CreateNot(TheBuilder->CreateFCmpOEQ(leftval, rightval));
    } else {
      return TheBuilder->CreateNot(TheBuilder->CreateICmpEQ(leftval, rightval));
    }
  } else if (op_ == "<") {
    if (floatop) {
      return TheBuilder->CreateFCmpOLT(leftval, rightval);
    } else {
      return TheBuilder->CreateICmpSLT(leftval, rightval);
    }
  } else if (op_ == ">") {
    if (floatop) {
      return TheBuilder->CreateFCmpOGT(leftval, rightval);
    } else {
      return TheBuilder->CreateICmpSGT(leftval, rightval);
    }
  } else if (op_ == "<=") {
    if (floatop) {
      return TheBuilder->CreateFCmpOLE(leftval, rightval);
    } else {
      return TheBuilder->CreateICmpSLE(leftval, rightval);
    }
  } else if (op_ == ">=") {
    if (floatop) {
      return TheBuilder->CreateFCmpOGE(leftval, rightval);
    } else {
      return TheBuilder->CreateICmpSGE(leftval, rightval);
    }
  } else {
    throw std::runtime_error("Unknown operator: " + op_);
  }

  return TheBuilder->CreateBinOp(op, leftval, rightval);
}

ASTCast::ASTCast(ASTNode *value, ASTType *type, unsigned long pos)
    : ASTNode(pos), value_(value), type_(type) {}

ASTCast::~ASTCast() {
  delete type_;
  delete value_;
}

llvm::Value *create_complex_cast(llvm::Value *val, llvm::Type *type_to) {
  llvm::Instruction::CastOps op = llvm::Instruction::CastOps::BitCast;
  auto type_from = val->getType();
  if (type_to->isPointerTy() && type_from->isPointerTy()) {
    op = llvm::Instruction::CastOps::BitCast;
  } else if (type_from->isFloatTy()) {
    if (type_to->isFloatTy()) {
      if (type_from->canLosslesslyBitCastTo(type_to)) {
        op = llvm::Instruction::CastOps::FPExt;
      } else {
        op = llvm::Instruction::CastOps::FPTrunc;
      }
    } else {
      op = llvm::Instruction::CastOps::FPToSI;
    }
  } else if (type_to->isFloatTy()) {
    op = llvm::Instruction::CastOps::SIToFP;
  } else if (type_from->canLosslesslyBitCastTo(type_to)) {
    op = llvm::Instruction::CastOps::SExt;
  } else {
    op = llvm::Instruction::CastOps::Trunc;
  }

  return TheBuilder->CreateCast(op, val, type_to);
}

llvm::Value *ASTCast::codegen(CompilerStackFrame *frame) {
  auto orig_val = value_->codegen(frame);
  auto type_to = type_->into_llvm_type();
  return create_complex_cast(orig_val, type_to);
}

ASTVariableDecl::ASTVariableDecl(const std::string &name, ASTType *type, ASTNode *value,
                unsigned long pos)
    : ASTNode(pos), name_(name), type_(type), value_(value) {}

ASTVariableDecl::~ASTVariableDecl() {
  delete type_;
  delete value_;
}

llvm::Value *ASTVariableDecl::codegen(CompilerStackFrame *frame) {
    auto varspace = TheBuilder->CreateAlloca(type_->into_llvm_type());
    frame->set(name_, varspace);
    if (value_) {
      auto val = value_->codegen(frame);
      TheBuilder->CreateStore(val, varspace);
    }
    //TheBuilder->CreateLifetimeStart(varspace);
    return llvm::PoisonValue::get(TheBuilder->getVoidTy());
}

ASTVariable::ASTVariable(const std::string &name, unsigned long pos)
    : ASTNode(pos), name_(name) {}

llvm::Value *ASTVariable::codegen(CompilerStackFrame *frame) {
  try {
    auto space = frame->resolve(name_);
    return TheBuilder->CreateLoad(space->getType()->getPointerElementType(), space);
  } catch (std::string _e) {
    //no variable found, get global scope data
    return Module->getFunction(name_);
  }
}

ASTVariableAssign::ASTVariableAssign(const std::string &name, ASTNode *value,
                  unsigned long pos)
    : ASTNode(pos), name_(name), value_(value) {}

ASTVariableAssign::~ASTVariableAssign() { delete value_; }

llvm::Value *ASTVariableAssign::codegen(CompilerStackFrame *frame) {
  auto varspace = frame->resolve(name_);
  auto val = create_complex_cast(value_->codegen(frame), varspace->getType()->getPointerElementType());

  TheBuilder->CreateStore(val, varspace);
  return val;
}


ASTFunctionCall::ASTFunctionCall(ASTNode *pointer, ASTNodeList *args, unsigned long pos)
    : ASTNode(pos), pointer_(pointer), args_(args) {}

ASTFunctionCall::~ASTFunctionCall() { delete args_; }



llvm::Value *ASTFunctionCall::codegen(CompilerStackFrame *frame) {
  //TODO: This feels like it is guaranteed to segfault. Do it anyways.
  auto funval_raw = pointer_->codegen(frame);
  auto the_function_type = llvm::cast<llvm::FunctionType>(funval_raw->getType()->getPointerElementType());
  std::vector<llvm::Value *> args_generated;
  for (auto arg : args_->nodes_) {
    args_generated.push_back(arg->codegen(frame));
  }
  
  return TheBuilder->CreateCall(the_function_type, funval_raw, args_generated);
}

ASTIf::ASTIf(ASTNode *condition, ASTNode *body, ASTNode *elseBody,
      unsigned long pos)
    : ASTNode(pos), condition_(condition), body_(body), elseBody_(elseBody) {}

ASTIf::~ASTIf() {
  delete condition_;
  delete body_;
  delete elseBody_;
}

llvm::Value *ASTIf::codegen(CompilerStackFrame *frame) {
  auto parent = TheBuilder->GetInsertBlock()->getParent();
  auto before_the_if = llvm::BasicBlock::Create(*TheContext, "", parent);
  TheBuilder->CreateBr(before_the_if);
  TheBuilder->SetInsertPoint(before_the_if);
  auto if_cond = condition_->codegen(frame);
  
  auto body = llvm::BasicBlock::Create(*TheContext, "", parent);
  auto body_else = llvm::BasicBlock::Create(*TheContext, "", parent);
  llvm::Value *body_ret;
  llvm::Value *else_ret;
  auto post = llvm::BasicBlock::Create(*TheContext, "", parent);
  TheBuilder->CreateCondBr(if_cond, body, body_else);
  TheBuilder->SetInsertPoint(body);
  {
    CompilerStackFrame frame_body(frame);
    body_ret = body_->codegen(&frame_body);
    TheBuilder->CreateBr(post);
  }
  TheBuilder->SetInsertPoint(body_else);
  {
    CompilerStackFrame frame_else(frame);
    if (elseBody_) {
      else_ret = elseBody_->codegen(&frame_else);
    }
    TheBuilder->CreateBr(post);
  }

  TheBuilder->SetInsertPoint(post);

  auto ty_t = body_ret->getType();
  auto ty_f = else_ret->getType();
  llvm::Value *return_val;
  if ((ty_t->getTypeID() != ty_f->getTypeID()) || ty_t->isVoidTy()) {
    return_val = llvm::PoisonValue::get(TheBuilder->getVoidTy());
  } else {
    auto phi = TheBuilder->CreatePHI(ty_t, 2);
    phi->addIncoming(body_ret, body);
    phi->addIncoming(else_ret, body_else);
    return_val = phi;
  }
  return return_val;
}

ASTWhile::ASTWhile(ASTNode *condition, ASTNode *body, unsigned long pos)
    : ASTNode(pos), condition_(condition), body_(body) {}

ASTWhile::~ASTWhile() {
  delete condition_;
  delete body_;
}

llvm::Value *ASTWhile::codegen(CompilerStackFrame *frame) {
  auto parent = TheBuilder->GetInsertBlock()->getParent();
  auto while_cond = llvm::BasicBlock::Create(*TheContext, "", parent);
  TheBuilder->CreateBr(while_cond);
  TheBuilder->SetInsertPoint(while_cond);
  auto the_condition = condition_->codegen(frame);
  auto while_body = llvm::BasicBlock::Create(*TheContext, "", parent);
  auto post_while = llvm::BasicBlock::Create(*TheContext, "", parent);
  TheBuilder->CreateCondBr(the_condition, while_body, post_while);

  TheBuilder->SetInsertPoint(while_body);
  {
    CompilerStackFrame while_frame(frame);
    body_->codegen(&while_frame);
  }
  TheBuilder->CreateBr(while_cond);

  TheBuilder->SetInsertPoint(post_while);
  return llvm::PoisonValue::get(TheBuilder->getVoidTy());
}

ASTReturn::ASTReturn(ASTNode *value, unsigned long pos)
    : ASTNode(pos), value_(value) {}

ASTReturn::~ASTReturn() { delete value_; }

llvm::Value *ASTReturn::codegen(CompilerStackFrame *frame) {
  auto val = value_->codegen(frame);
  if (val->getType()->isVoidTy()) {
    TheBuilder->CreateRetVoid();
  } else {
    TheBuilder->CreateRet(create_complex_cast(val, TheBuilder->getCurrentFunctionReturnType()));
  }
  auto deadcode = llvm::BasicBlock::Create(*TheContext, "_deadcode", TheBuilder->GetInsertBlock()->getParent());
  TheBuilder->SetInsertPoint(deadcode);
  
  return llvm::PoisonValue::get(TheBuilder->getVoidTy());
}

ASTRef::ASTRef(std::string &variable, unsigned long pos)
    : ASTNode(pos), variable_(variable) {}

llvm::Value *ASTRef::codegen(CompilerStackFrame *frame) {
  return frame->resolve(variable_);
}

ASTDeref::ASTDeref(ASTNode *value, unsigned long pos)
    : ASTNode(pos), value_(value) {}

ASTDeref::~ASTDeref() { delete value_; }

llvm::Value *ASTDeref::codegen(CompilerStackFrame *frame) {
  auto ptr = value_->codegen(frame);
  return TheBuilder->CreateLoad(ptr->getType()->getPointerElementType(), ptr);
}

ASTNull::ASTNull(unsigned long pos) : ASTNode(pos) {}

ASTNull::~ASTNull() {}

llvm::Value *ASTNull::codegen(CompilerStackFrame *frame) {
  return llvm::ConstantPointerNull::get(TheBuilder->getVoidTy()->getPointerTo());
}
ASTVoid::ASTVoid(unsigned long pos) : ASTNode(pos) {}

ASTVoid::~ASTVoid() {}

llvm::Value *ASTVoid::codegen(CompilerStackFrame *frame) {
  return llvm::PoisonValue::get(TheBuilder->getVoidTy());
}

ASTBool::ASTBool(bool value, unsigned long pos) : ASTNode(pos), value_(value) {}

ASTBool::~ASTBool() {}

llvm::Value *ASTBool::codegen(CompilerStackFrame *frame) {
  return TheBuilder->getInt1(value_);
}

ASTArrayAccess::ASTArrayAccess(ASTNode *array, ASTNode *index, unsigned long pos)
    : ASTNode(pos), array_(array), index_(index) {}
ASTArrayAccess::~ASTArrayAccess() {
  delete array_;
  delete index_;
}

llvm::Value *ASTArrayAccess::codegen(CompilerStackFrame *frame) {
  auto array_val = array_->codegen(frame);
  auto idx = index_->codegen(frame);
  return TheBuilder->CreateExtractElement(array_val, idx);
}

ASTForIn::ASTForIn(std::string &variable, ASTNode *array, ASTNode *body,
          unsigned long pos)
    : ASTNode(pos), variable_(variable), array_(array), body_(body) {}
ASTForIn::~ASTForIn() {
  delete array_;
  delete body_;
}

llvm::Value *ASTForIn::codegen(CompilerStackFrame *frame) {
  auto parent = TheBuilder->GetInsertBlock()->getParent();

  auto source_array = array_->codegen(frame);
  auto iterator_var = TheBuilder->CreateAlloca(TheBuilder->getInt64Ty());
  TheBuilder->CreateStore(TheBuilder->getInt64(0), iterator_var);

  auto array_comp_loop = llvm::BasicBlock::Create(*TheContext, "", parent);
  TheBuilder->CreateBr(array_comp_loop);
  CompilerStackFrame generation_frame(frame);
  generation_frame.set(variable_, TheBuilder->CreateGEP(source_array->getType()->getArrayElementType()->getPointerTo(), source_array, TheBuilder->CreateLoad(iterator_var->getAllocatedType(),iterator_var)));
  body_->codegen(&generation_frame);
  TheBuilder->CreateStore(TheBuilder->CreateAdd(TheBuilder->CreateLoad(iterator_var->getAllocatedType(),iterator_var), TheBuilder->getInt64(1)), iterator_var);
  auto array_comp_loop_end = llvm::BasicBlock::Create(*TheContext, "", parent);
  TheBuilder->CreateCondBr(
    TheBuilder->CreateICmpULT(TheBuilder->CreateLoad(iterator_var->getAllocatedType(),iterator_var), TheBuilder->getInt64(source_array->getType()->getArrayNumElements())),
    array_comp_loop,
    array_comp_loop_end
  );
  TheBuilder->SetInsertPoint(array_comp_loop_end);

  return llvm::PoisonValue::get(TheBuilder->getVoidTy());
}

ASTArrayComprehension::ASTArrayComprehension(std::string &variable, ASTNode *array, ASTNode *body,
                      unsigned long pos)
    : ASTNode(pos), variable_(variable), array_(array), body_(body) {}

ASTArrayComprehension::~ASTArrayComprehension() {
  delete array_;
  delete body_;
}

llvm::Value *ASTArrayComprehension::codegen(CompilerStackFrame *frame) {
  auto parent = TheBuilder->GetInsertBlock()->getParent();

  auto source_array = array_->codegen(frame);
  auto dest_array = TheBuilder->CreateAlloca(source_array->getType());
  auto iterator_var = TheBuilder->CreateAlloca(TheBuilder->getInt64Ty());
  TheBuilder->CreateStore(TheBuilder->getInt64(0), iterator_var);

  auto array_comp_loop = llvm::BasicBlock::Create(*TheContext, "", parent);
  TheBuilder->CreateBr(array_comp_loop);
  CompilerStackFrame generation_frame(frame);
  generation_frame.set(variable_, TheBuilder->CreateGEP(source_array->getType()->getArrayElementType()->getPointerTo(), source_array, TheBuilder->CreateLoad(iterator_var->getAllocatedType(),iterator_var)));
  auto result_val = body_->codegen(&generation_frame);
  TheBuilder->CreateStore(result_val, TheBuilder->CreateGEP(source_array->getType()->getArrayElementType()->getPointerTo(), dest_array, std::vector<llvm::Value *>({TheBuilder->getInt64(0), TheBuilder->CreateLoad(iterator_var->getAllocatedType(),iterator_var)})));
  TheBuilder->CreateStore(TheBuilder->CreateAdd(TheBuilder->CreateLoad(iterator_var->getAllocatedType(),iterator_var), TheBuilder->getInt64(1)), iterator_var);
  auto array_comp_loop_end = llvm::BasicBlock::Create(*TheContext, "", parent);
  TheBuilder->CreateCondBr(
    TheBuilder->CreateICmpULT(TheBuilder->CreateLoad(iterator_var->getAllocatedType(),iterator_var), TheBuilder->getInt64(source_array->getType()->getArrayNumElements())),
    array_comp_loop,
    array_comp_loop_end
  );
  TheBuilder->SetInsertPoint(array_comp_loop_end);
  return TheBuilder->CreateLoad(dest_array->getAllocatedType(), dest_array);
}

ASTRange::ASTRange(ASTNode *start, ASTNode *end, unsigned long pos)
    : ASTNode(pos), start_(start), end_(end) {}

ASTRange::~ASTRange() {
  delete start_;
  delete end_;
}

llvm::Value *ASTRange::codegen(CompilerStackFrame *frame) {
  auto parent = TheBuilder->GetInsertBlock()->getParent();

  auto start = start_->codegen(frame);
  auto end = end_->codegen(frame);
  auto len = TheBuilder->CreateSub(end, start);
  auto storage = TheBuilder->CreateAlloca(start->getType(), len);
  auto iterator_var = TheBuilder->CreateAlloca(start->getType());
  TheBuilder->CreateStore(0, iterator_var);
  auto loop_body = llvm::BasicBlock::Create(*TheContext, "", parent);
  TheBuilder->CreateBr(loop_body);
  TheBuilder->SetInsertPoint(loop_body);
  TheBuilder->CreateStore(TheBuilder->CreateAdd(TheBuilder->CreateLoad(iterator_var->getAllocatedType(), iterator_var), start), TheBuilder->CreateGEP(storage->getAllocatedType()->getArrayElementType()->getPointerTo(), storage, std::vector<llvm::Value *>({TheBuilder->getInt32(0), TheBuilder->CreateLoad(iterator_var->getAllocatedType(), iterator_var)})));
  TheBuilder->CreateStore(TheBuilder->CreateAdd(TheBuilder->CreateLoad(iterator_var->getAllocatedType(), iterator_var), TheBuilder->getInt64(1)), iterator_var);
  auto loop_end = llvm::BasicBlock::Create(*TheContext, "", parent);
  TheBuilder->CreateCondBr(
    TheBuilder->CreateICmpULT(TheBuilder->CreateLoad(iterator_var->getAllocatedType(), iterator_var), len),
    loop_body,
    loop_end
  );
  TheBuilder->SetInsertPoint(loop_end);
  return TheBuilder->CreateLoad(storage->getAllocatedType(), storage);
}

ASTFunctionType::ASTFunctionType(ASTType *return_type, std::vector<ASTType *> arg_types, unsigned long pos) : ASTType(pos), ret_type_(return_type), arg_types_(arg_types) {}

ASTFunctionType::~ASTFunctionType() {
  delete ret_type_;
  for (auto atype : arg_types_) {
    delete atype;
  }
}

llvm::Type *ASTFunctionType::into_llvm_type() {
  std::vector<llvm::Type *> args;
  args.reserve(arg_types_.size());
  for (auto arg : arg_types_) {
    args.push_back(arg->into_llvm_type());
  }
  return llvm::FunctionType::get(ret_type_->into_llvm_type(), args, false)->getPointerTo();
}