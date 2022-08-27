#include <cctype>
#include <cmath>
#include <exception>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <llvm/Passes/PassBuilder.h>
#include <llvm/Analysis/CGSCCPassManager.h>
#include <llvm/Analysis/LoopAnalysisManager.h>
#include <llvm/MC/TargetRegistry.h>
#include <llvm/ADT/APFloat.h>
#include <llvm/IR/Attributes.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/DataLayout.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalObject.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/Host.h>
#include <llvm/Support/CodeGen.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/IR/PassManager.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Target/TargetLoweringObjectFile.h>
#include <llvm/Target/TargetOptions.h>
#include <llvm/Target/TargetMachine.h>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <vector>
#include <sys/mman.h>

#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Module.h>

std::unique_ptr<llvm::LLVMContext> TheContext;
std::unique_ptr<llvm::IRBuilder<>> TheBuilder;
std::unique_ptr<llvm::Module> Module;

std::string indent(int level) {
    std::string s;
    for (int i = 0; i < level; ++i) {
      s += "  ";
    }
    return s;
  }

void init_module() {
  TheContext = std::make_unique<llvm::LLVMContext>();
  Module = std::make_unique<llvm::Module>("whale", *TheContext);
  TheBuilder = std::make_unique<llvm::IRBuilder<>>(*TheContext);
  }

class CompilerStackFrame {
  public:
    CompilerStackFrame(CompilerStackFrame *parent = nullptr) : parent(parent) {}
    void set(std::string name, llvm::Value *varspace) {
      data[name] = varspace;
    }
    virtual llvm::Value *resolve(std::string name) {
      if (data.count(name)) {
        return data[name];
      }
      if (parent) {
        //tail call optimization
        return parent->resolve(name);
      }
      throw "Undefined variable " + name;
    }
  protected:
  CompilerStackFrame *parent;
  std::map<std::string, llvm::Value *> data;
};

class LambdaStackFrame : public CompilerStackFrame {
  public:
    LambdaStackFrame(CompilerStackFrame *upper, std::function<llvm::Value *(std::string, llvm::Value *)> register_implicit) : parent(upper), implicit_handler(register_implicit) {};
    virtual llvm::Value *resolve(std::string name) {
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
  private:
  std::function<llvm::Value *(std::string name, llvm::Value *actual)> implicit_handler;
  protected:
  CompilerStackFrame *parent;
};

enum class ValueType {
  i32,
  i64,
  double_,
  float_,
  char_,
  void_,
  fun,
  bool_,
  pointer,
  template_,
  array
};
ValueType parseType(std::string type) {
  type = type.substr(0, type.find(':'));
  type = type.substr(0, type.find('-'));
  if (type == "i32") {
    return ValueType::i32;
  } else if (type == "i64") {
    return ValueType::i64;
  } else if (type == "double") {
    return ValueType::double_;
  } else if (type == "float") {
    return ValueType::float_;
  } else if (type == "char") {
    return ValueType::char_;
  } else if (type == "void") {
    return ValueType::void_;
  } else if (type == "fun") {
    return ValueType::fun;
  } else if (type == "bool") {
    return ValueType::bool_;
  } else if (type == "pointer") {
    return ValueType::pointer;
  } else if (type == "template") {
    return ValueType::template_;
  } else if (type == "array") {
    return ValueType::array;
  } else {
    throw std::runtime_error("Unknown type: " + type);
  }
}
size_t sizeOfType(std::string type) {
  ValueType typeVal = parseType(type);
  switch (typeVal) {
    case ValueType::i32:
      return sizeof(int);
      break;
    case ValueType::i64:
      return sizeof(long int);
      break;
    case ValueType::fun:
      return sizeof(void *);
      break;
    case ValueType::float_:
      return sizeof(float);
      break;
    case ValueType::double_:
      return sizeof(double);
      break;
    case ValueType::void_:
      return sizeof(char);
      break;
    case ValueType::bool_:
      return sizeof(bool);
      break;
    case ValueType::char_:
      return sizeof(char);
      break;
    case ValueType::pointer:
      return sizeof(void *);
      break;
    case ValueType::array:
      return sizeof(void *);
      break;
    default:
      throw std::runtime_error("Unknown type: " + type);
  }
}

class ASTNodeList;
class Runner;
class RunnerStackFrame;

class ASTNode {
 public:
  ASTNode(unsigned long pos) : pos_(pos) {}
  virtual ~ASTNode() {}
  virtual void print(std::ostream &out, int level = 0) const = 0;
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out = nullptr) const = 0;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const = 0;

  virtual llvm::Value *codegen(CompilerStackFrame *frame) = 0;
};
class ASTType {
  public:
    //TODO: Commenting this out causes many warnings but makes the code compile.
    //virtual ~ASTType();
    virtual llvm::Type *into_llvm_type() = 0;

    //weird interpreter / parser stuff stuff
    virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const = 0;
    virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out = nullptr) const = 0;

    virtual void print(std::ostream &out, int level = 0) const = 0;
};

class ASTBaseType : public ASTType {
 public:
  ASTBaseType(const std::string &name, unsigned long pos = 0)
      : ASTType(pos), name_(name) {}

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Type: " << name_ << std::endl;
  }
  virtual llvm::Type *into_llvm_type() {
    ValueType type = parseType(name_);
    switch (type) {
      case ValueType::i32:
        return llvm::Type::getInt32Ty(*TheContext);
      case ValueType::i64:
        return llvm::Type::getInt64Ty(*TheContext);
      case ValueType::fun:
        return llvm::Type::getPrimitiveType(*TheContext, llvm::Type::FunctionTyID);
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
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(int *)out = sizeOfType(name_);
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return name_;
  }

 private:
  std::string name_;
};
class ASTFunctionArg;
class Runner {
 public:
  bool _return = false;
  ASTNode *error = nullptr;
  Runner(ASTNode *ast, std::istream &code);

  void run(void *out);
  void *alloc(int size);

  void generateFunction(std::string name, std::vector<ASTFunctionArg *> args,
                        void (*body)(Runner *, RunnerStackFrame *),
                        ASTType *ret = new ASTBaseType("void"));

  void gc();

  void enterFrame();
  void exitFrame();

  void errorAt(const ASTNode *node, std::string msg);

 private:
  ASTNode *ast_;
  RunnerStackFrame *stackFrame_;
  std::istream &code_;
};



class RunnerStackFrame {
 public:
  RunnerStackFrame(RunnerStackFrame *parent = nullptr) : parent_(parent) {}
  ~RunnerStackFrame() {
    for (auto &kv : variables_) {
      free(kv.second);
    }
  }

  class Variable {
   public:
    Variable(void *ptr, ASTType *type) : ptr_(ptr), type_(type) {}
    void *ptr_;
    ASTType *type_;
  };

  Variable *getVariable(const std::string &name) {
    if (this->variables_[name] != nullptr) return this->variables_[name];
    if (this->parent_ != nullptr) return this->parent_->getVariable(name);
    throw std::runtime_error("Variable not found: " + name);
  };
  template <typename T>
  T getVariable(const std::string &name) {
    return *(T *)this->getVariable(name)->ptr_;
  };
  void *allocVariable(const std::string &name, ASTType *type, Runner *runner) {
    int size;
    type->run(runner, this, &size);
    void *value = runner->alloc(size);
    this->variables_[name] = new Variable(value, type);
    return value;
  };

  RunnerStackFrame *parent_;
  std::map<std::string, Variable *> variables_;
};

class ASTNodeList : public ASTNode {
 public:
  ASTNodeList(const std::vector<ASTNode *> &nodes, unsigned long pos = 0)
      : ASTNode(pos), nodes_(nodes) {}
  ASTNodeList(unsigned long pos = 0) : ASTNode(pos) {}

  virtual ~ASTNodeList() {
    for (auto node : nodes_) {
      delete node;
    }
  }

  void add(ASTNode *node) { nodes_.push_back(node); }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "NodeList:\n";
    for (auto node : nodes_) {
      node->print(out, level + 1);
    }
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    RunnerStackFrame newStackFrame(stackFrame);

    unsigned long trash_ = 0;
    for (auto node : nodes_) {
      node->run(runner, &newStackFrame, &trash_);
      if (runner->_return) {
        *(unsigned long *)out = trash_;
        break;
      }
    }
    runner->gc();
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }
  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    llvm::Value *retvar = llvm::PoisonValue::get(llvm::Type::getVoidTy(*TheContext));
    for (auto node : nodes_) {
      retvar = node->codegen(frame);
    }
    return retvar;
  }

  std::vector<ASTNode *> nodes_;

 private:
};
class ASTArrayLiteral : public ASTType {
 public:
  ASTArrayLiteral(ASTNodeList *list, unsigned long pos = 0)
      : ASTType(pos), list_(list) {}
  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "ArrayLiteral: " << std::endl;
    list_->print(out, level + 1);
  }
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    std::string type = "";

    for (auto node : this->list_->nodes_) {
      if (type == "") {
        type = node->returnType(runner, stackFrame);
      } else {
        if (type != node->returnType(runner, stackFrame)) {
          runner->errorAt(node,
                          "Array elements must be of the same type: " + type +
                              " != " + node->returnType(runner, stackFrame));
        }
      }
    }

    int size = sizeOfType(type);
    int allocs = size * this->list_->nodes_.size();
    void *value = runner->alloc(allocs);
    for (unsigned long i = 0; i < this->list_->nodes_.size(); ++i) {
      this->list_->nodes_[i]->run(runner, stackFrame, (char *)value + i * size);
    }

    *(void **)out = value;
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    std::string type = "";

    for (auto node : this->list_->nodes_) {
      if (type == "") {
        type = node->returnType(runner, stack);
      } else {
        if (type != node->returnType(runner, stack)) {
          runner->errorAt(node,
                          "Array elements must be of the same type: " + type +
                              " != " + node->returnType(runner, stack));
        }
      }
    }

    return "array-" + std::to_string(list_->nodes_.size()) + ":" + type;
  }

 private:
  ASTNodeList *list_;
};

class ASTTemplate : public ASTType {
 public:
  ASTTemplate(ASTType *type, const std::string &name, unsigned long pos = 0)
      : ASTType(pos), type_(type), name_(name) {}

  virtual llvm::Type *into_llvm_type() {
    return nullptr;
  }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Template: " << name_ << std::endl;
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(int *)out = sizeof(void *);  // templates always have a pointer size
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "template-" + name_ + ":" + type_->returnType(runner, stack);
  }

 private:
  ASTType *type_;
  std::string name_;
};

class ASTFunctionArg {
 public:
  ASTFunctionArg(ASTType *type, const std::string &name)
      : type_(type), name_(name) {}

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Argument: " << name_ << std::endl;
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    void *ptr = stackFrame->allocVariable(name_, (ASTType *)type_, runner);
    *(void **)out = ptr;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return type_->returnType(runner, stack);
  }
  ASTType *type_;
  std::string name_;
};

class ASTLambda : public ASTNode {
 public:
  ASTLambda(std::vector<ASTFunctionArg *> args, ASTNode *body, ASTType *type, unsigned long pos = 0)
      : args_(args), body_(body), type_(type) {}

  virtual ~ASTLambda() { delete body_; }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Lambda: " << std::endl;
    for (auto arg : args_) {
      arg->print(out, level + 1);
    }
    body_->print(out, level + 1);
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(const ASTLambda **)out = this;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "fun";
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    std::vector<llvm::Type *> implicit_args;
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
    
    auto lambda_body = llvm::BasicBlock::Create(*TheContext);
    
    auto where_we_left_off = TheBuilder->saveIP();

    
    LambdaStackFrame cur_frame(frame, [&](std::string name, llvm::Value *outer_definition) -> llvm::Value * {
      auto old_ip = TheBuilder->saveIP();

      TheBuilder->SetInsertPoint(lambda_setup_block);
      auto alloc_type = outer_definition->getType();
      auto proxy_pointer = TheBuilder->CreateGEP(alloc_type, lambda_func->getArg(0), std::vector<llvm::Value *>({TheBuilder->getInt32(0), TheBuilder->getInt32(implicit_args.size())}));
      implicit_args.push_back(alloc_type);
      trampoline_object_members.push_back(outer_definition);

      TheBuilder->restoreIP(old_ip);
      return proxy_pointer;
    });
    TheBuilder->SetInsertPoint(lambda_setup_block);
    for (int i=0;i<this->args_.size();i++) {
      auto normal_arg_memory = TheBuilder->CreateAlloca(total_args[i+1]);
      TheBuilder->CreateStore(lambda_func->getArg(i+1), normal_arg_memory);
      cur_frame.set(this->args_[i]->name_, normal_arg_memory);
    }
    TheBuilder->CreateBr(lambda_body);
    TheBuilder->SetInsertPoint(lambda_body);
    body_->codegen(&cur_frame);
  
    TheBuilder->restoreIP(where_we_left_off);
    lambda_trampoline_data->setBody(implicit_args);
    auto the_trampoline_object = TheBuilder->CreateAlloca(lambda_trampoline_data);
    //load our trampoline
    for (int i = 0; i < trampoline_object_members.size(); i++) {
      TheBuilder->CreateStore(trampoline_object_members[i], TheBuilder->CreateGEP(implicit_args[i], the_trampoline_object, std::vector<llvm::Value *>({TheBuilder->getInt32(0), TheBuilder->getInt32(i)})));
    }
    //create trampoline intrinsic
    //TODO unmapping this memory sometime is something to be considered.
    auto tramp_memory = TheBuilder->CreateCall(Module->getFunction("mmap"), std::vector<llvm::Value *>({TheBuilder->CreateCast(llvm::Instruction::BitCast, TheBuilder->getInt64(0), TheBuilder->getInt8PtrTy()), TheBuilder->getInt64(72), TheBuilder->getInt32(PROT_READ | PROT_WRITE | PROT_EXEC), TheBuilder->getInt32(MAP_ANONYMOUS | MAP_PRIVATE), TheBuilder->getInt32(0), TheBuilder->getInt64(0)}));
    TheBuilder->CreateIntrinsic(llvm::Intrinsic::init_trampoline, std::vector<llvm::Type *>({TheBuilder->getInt8PtrTy(), TheBuilder->getInt8PtrTy(), TheBuilder->getInt8PtrTy()}), std::vector<llvm::Value *>({tramp_memory, lambda_func, the_trampoline_object}));
    auto the_final_func_ptr = TheBuilder->CreateUnaryIntrinsic(llvm::Intrinsic::adjust_trampoline, tramp_memory);
    total_args.erase(total_args.begin());
    return TheBuilder->CreateCast(llvm::Instruction::BitCast, the_final_func_ptr, llvm::FunctionType::get(type_->into_llvm_type(), total_args, false));
  }

  std::vector<ASTFunctionArg *> args_;
  ASTNode *body_;
  ASTType *type_;

 private:
};

class ASTNativeFunction : public ASTNode {
 public:
  ASTNativeFunction(void (*function)(Runner *, RunnerStackFrame *),
                    unsigned long pos = 0)
      : ASTNode(pos), function_(function) {}

  virtual ~ASTNativeFunction() {}

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "native @ " << function_;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    function_(runner, stackFrame);
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    throw "No compiler-native functions in LLVM compilation allowed!";
  }

 private:
  void (*function_)(Runner *, RunnerStackFrame *);
};

void errorWithMessage(const std::string &msg, std::istream &code_,
                      unsigned long pos) {
  int line = 1;
  int column = 0;
  std::string lineString;
  code_.clear();
  code_.seekg(0);
  for (unsigned long i = 0; i < pos; i++) {
    if (code_.peek() == EOF) {
      throw std::runtime_error("Error at end of file: " + msg);
    }
    if (code_.peek() == '\n') {
      line++;
      column = 0;
    } else {
      column++;
    }
    code_.get();
  }
  code_.clear();
  code_.seekg(0);
  for (int i = 0; i < line; i++) {
    std::getline(code_, lineString);
  }

  std::string lineNoString = std::to_string(line);

  std::cerr << "Error at " << line << ":" << column << "\t" << msg << "\n\n"
            << std::endl;
  std::cerr << std::string(5 - std::min<int>(5, lineNoString.length()), ' ') +
                   lineNoString
            << " | " << lineString << std::endl;
  std::cerr << "      | " << std::string(std::max(column - 1, 0), '~') << "^"
            << "\n\n"
            << std::endl;
  exit(1);
}

Runner::Runner(ASTNode *ast, std::istream &code) : ast_(ast), code_(code) {
  code_.clear();
  code_.seekg(0);
  this->stackFrame_ = new RunnerStackFrame();
}
void Runner::run(void *out) { ast_->run(this, this->stackFrame_, out); }
void *Runner::alloc(int size) { return malloc(size); }

void Runner::generateFunction(std::string name, std::vector<ASTFunctionArg *> args,
                              void (*body)(Runner *, RunnerStackFrame *),
                              ASTType *ret) {
  ASTLambda *newLambda =
      new ASTLambda(args, new ASTNodeList({new ASTNativeFunction(body)}), ret);
  ASTType *newTemplate = new ASTBaseType("fun");
  void *ptr = this->stackFrame_->allocVariable(name, newTemplate, this);
  newLambda->run(this, stackFrame_, ptr);
}
void Runner::gc() {}
void Runner::errorAt(const ASTNode *node, std::string message) {
  errorWithMessage(message, code_, node->pos_);
}

class ASTPointer : public ASTType {
 public:
  ASTPointer(ASTType *type, unsigned long pos = 0)
      : ASTType(pos), type_(type) {}

  virtual llvm::Type *into_llvm_type() {
    return this->type_->into_llvm_type()->getPointerTo();
  }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Pointer: " << std::endl;
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(int *)out = sizeof(void *);
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "pointer:" + type_->returnType(runner, stack);
  }

 private:
  ASTType *type_;
};
class ASTArray : public ASTType {
 public:
  ASTArray(ASTType *type, ASTNode *size, unsigned long pos = 0)
      : ASTType(pos), type_(type), size_(size) {}

  virtual llvm::Type *into_llvm_type() {
    return this->type_->into_llvm_type()->getPointerTo();
  }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Array: " << std::endl;
    type_->print(out, level + 1);
    size_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    int sizeOutput, typeOutput;
    size_->run(runner, stackFrame, &sizeOutput);
    type_->run(runner, stackFrame, &typeOutput);
    *(int *)out = sizeOutput * typeOutput;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    int sizeOutput;
    size_->run(runner, stack, &sizeOutput);
    return "array-" + std::to_string(sizeOutput) + ":" +
           type_->returnType(runner, stack);
  }

  ASTType *type_;

 private:
  ASTNode *size_;
};

class ASTFunctionArg : public ASTNode {
 public:
  ASTFunctionArg(ASTType *type, const std::string &name, unsigned long pos = 0)
      : ASTNode(pos), name_(name), type_(type) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Argument: " << name_ << std::endl;
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    void *ptr = stackFrame->allocVariable(name_, (ASTType *)type_, runner);
    *(void **)out = ptr;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return type_->returnType(runner, stack);
  }
  std::string name_;

 private:
  ASTType *type_;
};

class ASTNumber : public ASTNode {
 public:
  ASTNumber(long int value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Number: " << value_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(long int *)out = value_;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "i64";
  }
  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    return TheBuilder->getInt64(value_);
  }

 private:
  long int value_;
};
class ASTChar : public ASTNode {
 public:
  ASTChar(char value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}
  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Char: " << value_ << std::endl;
  }
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(char *)out = value_;
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "char";
  }
 private:
  char value_;
};
class ASTDouble : public ASTNode {
 public:
  ASTDouble(double value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Double: " << value_ << std::endl;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "double";
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(double *)out = value_;
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    return llvm::ConstantFP::get(TheBuilder->getDoubleTy(), value_);
  }

 private:
  double value_;
};
class ASTString : public ASTNode {
 public:
  ASTString(const std::string &value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "String: \"" << value_ << "\"" << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(const char **)out = value_.c_str();
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "pointer:char";
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    return TheBuilder->CreateGlobalStringPtr(value_);
  }

 private:
  std::string value_;
};

class ASTBinaryOp : public ASTNode {
 public:
  ASTBinaryOp(std::string op, ASTNode *left, ASTNode *right,
              unsigned long pos = 0)
      : ASTNode(pos), op_(op), left_(left), right_(right) {}

  virtual ~ASTBinaryOp() {
    delete left_;
    delete right_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "BinaryOp: " << op_ << std::endl;
    left_->print(out, level + 1);
    right_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    int leftOutput, rightOutput;
    left_->run(runner, stackFrame, &leftOutput);
    right_->run(runner, stackFrame, &rightOutput);

    double leftOutputDouble, rightOutputDouble;
    left_->run(runner, stackFrame, &leftOutputDouble);
    right_->run(runner, stackFrame, &rightOutputDouble);

    if (op_ == "+") {
      *(int *)out = leftOutput + rightOutput;
    } else if (op_ == "-") {
      *(int *)out = leftOutput - rightOutput;
    } else if (op_ == "*") {
      *(int *)out = leftOutput * rightOutput;
    } else if (op_ == "/") {
      *(double *)out = leftOutputDouble / rightOutputDouble;
    } else if (op_ == "//") {
      *(int *)out = leftOutput / rightOutput;
    } else if (op_ == "%") {
      *(int *)out = leftOutput % rightOutput;
    } else if (op_ == "<<") {
      *(int *)out = leftOutput << rightOutput;
    } else if (op_ == ">>") {
      *(int *)out = leftOutput >> rightOutput;
    } else if (op_ == "&") {
      *(int *)out = leftOutput & rightOutput;
    } else if (op_ == "|") {
      *(int *)out = leftOutput | rightOutput;
    } else if (op_ == "^") {
      *(int *)out = leftOutput ^ rightOutput;
    } else if (op_ == "&&") {
      *(int *)out = leftOutput && rightOutput;
    } else if (op_ == "||") {
      *(int *)out = leftOutput || rightOutput;
    } else if (op_ == "==") {
      *(int *)out = leftOutput == rightOutput;
    } else if (op_ == "!=") {
      *(int *)out = leftOutput != rightOutput;
    } else if (op_ == "<") {
      *(int *)out = leftOutput < rightOutput;
    } else if (op_ == ">") {
      *(int *)out = leftOutput > rightOutput;
    } else if (op_ == "<=") {
      *(int *)out = leftOutput <= rightOutput;
    } else if (op_ == ">=") {
      *(int *)out = leftOutput >= rightOutput;
    } else {
      runner->errorAt(this, "Unknown binary operator: " + op_);
    }
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
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

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return op_ == "/" ? "double" : "i64";
  }

 private:
  std::string op_;
  ASTNode *left_;
  ASTNode *right_;
};

class ASTCast : public ASTNode {
 public:
  ASTCast(ASTNode *value, ASTType *type, unsigned long pos = 0)
      : ASTNode(pos), value_(value), type_(type) {}

  virtual ~ASTCast() {
    delete type_;
    delete value_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Cast: " << std::endl;
    type_->print(out, level + 1);
    value_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    ValueType type = parseType(value_->returnType(runner, stackFrame));
    ValueType castType = parseType(type_->returnType(runner, stackFrame));

    if ((type == ValueType::i32 || type == ValueType::i64) &&
        castType == ValueType::double_) {
      long int value;
      value_->run(runner, stackFrame, &value);
      *(double *)out = (double)value;
    } else if (type == ValueType::double_ &&
               (castType == ValueType::i32 || castType == ValueType::i64)) {
      double value;
      value_->run(runner, stackFrame, &value);
      *(long int *)out = (long int)value;
    } else if (type == ValueType::i32 && castType == ValueType::i64) {
      long int value;
      value_->run(runner, stackFrame, &value);
      *(long int *)out = value;
    } else if (type == ValueType::i64 && castType == ValueType::i32) {
      long int value;
      value_->run(runner, stackFrame, &value);
      *(long int *)out = value;
    } else if (type == ValueType::pointer && castType == ValueType::i64) {
      void *value;
      value_->run(runner, stackFrame, &value);
      *(long int *)out = (long int)value;
    } else if (type == ValueType::i64 && castType == ValueType::pointer) {
      long int value;
      value_->run(runner, stackFrame, &value);
      *(void **)out = (void *)value;
    } else if (type == ValueType::char_ && castType == ValueType::i64) {
      char value;
      value_->run(runner, stackFrame, &value);
      *(long int *)out = (long int)value;
    } else if (type == ValueType::i64 && castType == ValueType::char_) {
      long int value;
      value_->run(runner, stackFrame, &value);
      *(char *)out = (char)value;
    }

    else {
      runner->errorAt(
          this, "Unknown cast: " + value_->returnType(runner, stackFrame) +
                    " to " + type_->returnType(runner, stackFrame));
    }
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return type_->returnType(runner, stack);
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    llvm::Instruction::CastOps op = llvm::Instruction::CastOps::BitCast;
    auto orig_val = value_->codegen(frame);
    auto type_from = orig_val->getType();
    auto type_to = type_->into_llvm_type();
    if (type_to->isPointerTy() && type_from->isPointerTy()) {
      //bitcast will suffice
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

    return TheBuilder->CreateCast(op, orig_val, type_to);
  }

 private:
  ASTNode *value_;
  ASTType *type_;
};

class ASTVariableDecl : public ASTNode {
 public:
  ASTVariableDecl(const std::string &name, ASTType *type, ASTNode *value,
                  unsigned long pos = 0)
      : ASTNode(pos), name_(name), type_(type), value_(value) {}

  virtual ~ASTVariableDecl() {
    delete type_;
    delete value_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "VariableDecl: " << name_ << std::endl;
    type_->print(out, level + 1);
    if (value_ != nullptr) {
      value_->print(out, level + 1);
    }
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    if (value_ != nullptr) {
      if (type_->returnType(runner, stackFrame) !=
          value_->returnType(runner, stackFrame)) {
        runner->errorAt(
            this, "Type mismatch: " + type_->returnType(runner, stackFrame) +
                      " != " + value_->returnType(runner, stackFrame));
      }
    }

    void *ptr = stackFrame->allocVariable(name_, (ASTType *)type_, runner);
    if (value_ != nullptr) {
      value_->run(runner, stackFrame, ptr);
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }
  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    auto varspace = TheBuilder->CreateAlloca(type_->into_llvm_type());
    frame->set(name_, varspace);
    return llvm::PoisonValue::get(TheBuilder->getVoidTy());
  }

 private:
  std::string name_;
  ASTType *type_;
  ASTNode *value_;
};
class ASTVariable : public ASTNode {
 public:
  ASTVariable(const std::string &name, unsigned long pos = 0)
      : ASTNode(pos), name_(name) {}

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Variable: " << name_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    auto it = stackFrame->getVariable(name_);

    if (out != nullptr) {
      *(unsigned long *)out = (unsigned long)*(unsigned long int **)(it->ptr_);
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return stack->getVariable(name_)->type_->returnType(runner, stack);
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    try {
      auto space = frame->resolve(name_);
      return TheBuilder->CreateLoad(space->getType()->getPointerElementType(), space);
    } catch (std::string _e) {
      //no variable found, get global scope data
      return Module->getFunction(name_);
    }
  }

 private:
  std::string name_;
};
class ASTVariableAssign : public ASTNode {
 public:
  ASTVariableAssign(const std::string &name, ASTNode *value,
                    unsigned long pos = 0)
      : ASTNode(pos), name_(name), value_(value) {}

  virtual ~ASTVariableAssign() { delete value_; }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "VariableAssign: " << name_ << std::endl;
    value_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    auto it = stackFrame->getVariable(name_)->ptr_;
    value_->run(runner, stackFrame, it);
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }
  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    auto varspace = frame->resolve(name_);
    auto val = value_->codegen(frame);
    TheBuilder->CreateStore(val, varspace);
    return val;
  }

 private:
  std::string name_;
  ASTNode *value_;
};
class ASTFunctionCall : public ASTNode {
 public:
  ASTFunctionCall(ASTNode *pointer, ASTNodeList *args, unsigned long pos = 0)
      : ASTNode(pos), pointer_(pointer), args_(args) {}

  virtual ~ASTFunctionCall() { delete args_; }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "FunctionCall: " << std::endl;
    pointer_->print(out, level + 1);
    args_->print(out, level + 1);
  }

  ASTLambda *getLambda(Runner *runner, RunnerStackFrame *stackFrame) const {
    void *lambdaAddress;
    pointer_->run(runner, stackFrame, &lambdaAddress);

    ASTLambda *lambda = (ASTLambda *)lambdaAddress;
    return lambda;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    ASTLambda *lambda = getLambda(runner, stackFrame);
    RunnerStackFrame newStackFrame(stackFrame);
    int i = 0;
    if (lambda->args_.size() != args_->nodes_.size()) {
      runner->errorAt(this, "Wrong number of arguments on function call: " +
                               std::to_string(lambda->args_.size()) +
                               " expected, " +
                               std::to_string(args_->nodes_.size()) + " given");
    }
    for (auto arg : lambda->args_) {
      if (arg->returnType(runner, &newStackFrame) !=
          args_->nodes_[i]->returnType(runner, &newStackFrame)) {
        runner->errorAt(
            this, "Type mismatch on function argument" +
                      ((ASTFunctionArg *)arg)->name_ + ": " +
                      arg->returnType(runner, &newStackFrame) + " expected, " +
                      args_->nodes_[i]->returnType(runner, &newStackFrame) +
                      " given");
      }

      void *variableAddress;
      arg->run(runner, &newStackFrame, &variableAddress);
      this->args_->nodes_[i]->run(runner, stackFrame, variableAddress);
      i++;
    }

    int bytes;
    lambda->type_->run(runner, &newStackFrame, &bytes);
    void *returnAddress = runner->alloc(bytes);

    lambda->body_->run(runner, &newStackFrame, returnAddress);

    if (out != nullptr) {
      *(unsigned long *)out = *(unsigned long *)returnAddress;
    }

    runner->_return = false;

    runner->gc();
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return getLambda(runner, stack)->type_->returnType(runner, stack);
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    //TODO: This feels like it is guaranteed to segfault. Do it anyways.
    auto fp = static_cast<llvm::Function *>(pointer_->codegen(frame));
    std::vector<llvm::Value *> args_generated;
    for (auto arg : args_->nodes_) {
      args_generated.push_back(arg->codegen(frame));
    }
    return TheBuilder->CreateCall(fp, args_generated);
  }

 private:
  ASTNode *pointer_;
  ASTNodeList *args_;
};
class ASTIf : public ASTNode {
 public:
  ASTIf(ASTNode *condition, ASTNode *body, ASTNode *elseBody,
        unsigned long pos = 0)
      : ASTNode(pos), condition_(condition), body_(body), elseBody_(elseBody) {}

  virtual ~ASTIf() {
    delete condition_;
    delete body_;
    delete elseBody_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "If: " << std::endl;
    condition_->print(out, level + 1);
    body_->print(out, level + 1);
    if (elseBody_ != nullptr) {
      elseBody_->print(out, level + 1);
    }
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    int conditionOutput;
    condition_->run(runner, stackFrame, &conditionOutput);
    if (conditionOutput) {
      body_->run(runner, stackFrame, out);
    } else if (elseBody_ != nullptr) {
      elseBody_->run(runner, stackFrame, out);
    }
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    auto before_the_if = llvm::BasicBlock::Create(*TheContext);
    auto common_var_setup_point = TheBuilder->CreateBr(before_the_if);
    TheBuilder->SetInsertPoint(before_the_if);
    auto if_cond = condition_->codegen(frame);
    
    auto body = llvm::BasicBlock::Create(*TheContext);
    auto body_else = llvm::BasicBlock::Create(*TheContext);
    llvm::Value *body_ret;
    llvm::Value *else_ret;
    auto post = llvm::BasicBlock::Create(*TheContext);
    TheBuilder->CreateCondBr(if_cond, body, body_else);
    TheBuilder->SetInsertPoint(body);
    {
      CompilerStackFrame frame_body(frame);
      body_ret = body_->codegen(&frame_body);
    }
    TheBuilder->SetInsertPoint(body_else);
    {
      CompilerStackFrame frame_else(frame);
      else_ret = body_->codegen(&frame_else);
    }
    auto ty_t = body_ret->getType();
    auto ty_f = else_ret->getType();
    llvm::Value *return_val;
    if ((ty_t->getTypeID() != ty_f->getTypeID()) || ty_t->isVoidTy()) {
      return_val = llvm::PoisonValue::get(TheBuilder->getVoidTy());
    } else {
      TheBuilder->SetInsertPoint(common_var_setup_point);
      auto rvar = TheBuilder->CreateAlloca(ty_t);
      TheBuilder->SetInsertPoint(body);
      TheBuilder->CreateStore(body_ret, rvar);
      TheBuilder->SetInsertPoint(body_else);
      TheBuilder->CreateStore(else_ret, rvar);
      TheBuilder->SetInsertPoint(post);
      return_val = TheBuilder->CreateLoad(ty_t,rvar);
    }
    TheBuilder->SetInsertPoint(body);
    TheBuilder->CreateBr(post);
    TheBuilder->SetInsertPoint(body_else);
    TheBuilder->CreateBr(post);
    TheBuilder->SetInsertPoint(post);
    return return_val;
  }

 private:
  ASTNode *condition_;
  ASTNode *body_;
  ASTNode *elseBody_;
};
class ASTWhile : public ASTNode {
 public:
  ASTWhile(ASTNode *condition, ASTNode *body, unsigned long pos = 0)
      : ASTNode(pos), condition_(condition), body_(body) {}

  virtual ~ASTWhile() {
    delete condition_;
    delete body_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "While: " << std::endl;
    condition_->print(out, level + 1);
    body_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    int conditionOutput;
    condition_->run(runner, stackFrame, &conditionOutput);
    while (conditionOutput) {
      body_->run(runner, stackFrame, out);
      condition_->run(runner, stackFrame, &conditionOutput);
    }
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    auto while_cond = llvm::BasicBlock::Create(*TheContext);
    TheBuilder->CreateBr(while_cond);
    TheBuilder->SetInsertPoint(while_cond);
    auto the_condition = condition_->codegen(frame);
    auto while_body = llvm::BasicBlock::Create(*TheContext);
    auto post_while = llvm::BasicBlock::Create(*TheContext);
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

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

 private:
  ASTNode *condition_;
  ASTNode *body_;
};
class ASTReturn : public ASTNode {
 public:
  ASTReturn(ASTNode *value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}

  virtual ~ASTReturn() { delete value_; }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Return: " << std::endl;
    if (value_ != nullptr) {
      value_->print(out, level + 1);
    }
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    if (value_) {
      value_->run(runner, stackFrame, out);
    }
    runner->_return = true;
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }
  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    TheBuilder->CreateRet(value_->codegen(frame));
    return llvm::PoisonValue::get(TheBuilder->getVoidTy());
  }

 private:
  ASTNode *value_;
};
class ASTRef : public ASTNode {
 public:
  ASTRef(std::string &variable, unsigned long pos = 0)
      : ASTNode(pos), variable_(variable) {}

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Ref: " << variable_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    void *variableAddress = stackFrame->getVariable(variable_)->ptr_;
    if (out != nullptr) {
      *(unsigned long **)out = (unsigned long *)variableAddress;
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "pointer:" +
           stack->getVariable(variable_)->type_->returnType(runner, stack);
  }
  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    return frame->resolve(variable_);
  }

 private:
  std::string variable_;
};
class ASTDeref : public ASTNode {
 public:
  ASTDeref(ASTNode *value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}

  virtual ~ASTDeref() { delete value_; }

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Deref: " << std::endl;
    value_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    if (value_->returnType(runner, stackFrame).substr(0, 8) != "pointer:") {
      runner->errorAt(this, "Cannot dereference non-pointer type");
    }

    void *value;
    value_->run(runner, stackFrame, &value);
    if (out != nullptr) {
      *(unsigned long *)out = *(unsigned long *)value;
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    if (value_->returnType(runner, stack).substr(0, 8) == "pointer:") {
      return value_->returnType(runner, stack).substr(8);
    } else {
      runner->errorAt(this, "Cannot dereference non-pointer type");
      return "void";
    }
  }

  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    auto ptr = value_->codegen(frame);
    return TheBuilder->CreateLoad(ptr->getType()->getPointerElementType(), ptr);
  }

 private:
  ASTNode *value_;
};
class ASTNull : public ASTNode {
 public:
  ASTNull(unsigned long pos = 0) : ASTNode(pos) {}

  virtual ~ASTNull() {}

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Null" << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    if (out != nullptr) {
      *(unsigned long *)out = 0;
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "pointer:void";
  }
  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    return llvm::ConstantPointerNull::get(TheBuilder->getVoidTy()->getPointerTo());
  }

 private:
};
class ASTBool : public ASTNode {
 public:
  ASTBool(bool value, unsigned long pos = 0) : ASTNode(pos), value_(value) {}

  virtual ~ASTBool() {}

  virtual void print(std::ostream &out, int level) const {
    out << indent(level) << "Bool: " << value_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    if (out != nullptr) {
      *(bool *)out = value_;
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "bool";
  }
  virtual llvm::Value *codegen(CompilerStackFrame *frame) {
    return TheBuilder->getInt1(value_);
  }

 private:
  bool value_;
};
class ASTArrayAccess : public ASTNode {
 public:
  ASTArrayAccess(ASTNode *array, ASTNode *index, unsigned long pos = 0)
      : ASTNode(pos), array_(array), index_(index) {}
  virtual ~ASTArrayAccess() {
    delete array_;
    delete index_;
  }
  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "ArrayAccess: " << std::endl;
    array_->print(out, level + 1);
    index_->print(out, level + 1);
  }
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    if (array_->returnType(runner, stackFrame).substr(0, 6) != "array-" &&
        array_->returnType(runner, stackFrame).substr(0, 7) != "pointer") {
      runner->errorAt(this, "Cannot access non-accessible type");
    }

    std::string arrayType = array_->returnType(runner, stackFrame);
    arrayType = arrayType.substr(arrayType.find(":") + 1);

    void *array, *array2;
    array_->run(runner, stackFrame, &array);
    array2 = array;  // there is a bug here
    int index;
    index_->run(runner, stackFrame, &index);
    if (out != nullptr) {
      std::memcpy(out, (char *)array2 + index * sizeOfType(arrayType),
                  sizeOfType(arrayType));
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    if (array_->returnType(runner, stack).substr(0, 6) != "array-" &&
        array_->returnType(runner, stack).substr(0, 7) != "pointer") {
      runner->errorAt(this, "Cannot access non-accessible type");
    }

    return array_->returnType(runner, stack)
        .substr(array_->returnType(runner, stack).find(':') + 1);
  }

 private:
  ASTNode *array_;
  ASTNode *index_;
};

ASTType *typeFromTypeRep(std::string type) {
  if (type.substr(0, 6) == "array-") {
    return new ASTArray(
        typeFromTypeRep(type.substr(type.find(':') + 1)),
        new ASTNumber(std::stoi(type.substr(
            type.find('-') + 1, type.find(':') - type.find('-') - 1))));
  } else if (type.substr(0, 7) == "pointer") {
    return new ASTPointer(typeFromTypeRep(type.substr(type.find(':') + 1)));
  } else if (type.substr(0, 9) == "template-") {
    return new ASTTemplate(
        typeFromTypeRep(type.substr(type.find(':') + 1)),
        type.substr(type.find('-') + 1, type.find(':') - type.find('-') - 1));
  } else {
    return new ASTBaseType(type);
  }
}

class ASTForIn : public ASTNode {
 public:
  ASTForIn(std::string &variable, ASTNode *array, ASTNode *body,
           unsigned long pos = 0)
      : ASTNode(pos), variable_(variable), array_(array), body_(body) {}
  virtual ~ASTForIn() {
    delete array_;
    delete body_;
  }
  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "ForIn: " << variable_ << std::endl;
    array_->print(out, level + 1);
    body_->print(out, level + 1);
  }
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    std::string arrayReturn = array_->returnType(runner, stackFrame);
    if (arrayReturn.substr(0, 6) != "array-") {
      runner->errorAt(this, "Cannot iterate non-array type");
    }
    int arrayLength = std::stoi(
        arrayReturn.substr(arrayReturn.find('-') + 1,
                           arrayReturn.find(':') - arrayReturn.find('-') - 1));
    void *array, *array2;
    array_->run(runner, stackFrame, &array);
    array2 = array;  // there is a bug here
    RunnerStackFrame newStackFrame(stackFrame);

    ASTType *elementType =
        typeFromTypeRep(arrayReturn.substr(arrayReturn.find(':') + 1));
    void *forOut = newStackFrame.allocVariable(variable_, elementType, runner);
    for (int i = 0; i < arrayLength; i++) {
      std::memcpy(forOut,
                  (char *)array2 + i * sizeOfType(arrayReturn.substr(
                                           arrayReturn.find(":") + 1)),
                  sizeOfType(arrayReturn.substr(arrayReturn.find(":") + 1)));
      body_->run(runner, &newStackFrame);
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

 private:
  std::string variable_;
  ASTNode *array_;
  ASTNode *body_;
};
class ASTArrayComprehension : public ASTNode {
 public:
  ASTArrayComprehension(std::string &variable, ASTNode *array, ASTNode *body,
                        unsigned long pos = 0)
      : ASTNode(pos), variable_(variable), array_(array), body_(body) {}
  virtual ~ASTArrayComprehension() {
    delete array_;
    delete body_;
  }
  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "ArrayComprehension: " << variable_
        << std::endl;
    array_->print(out, level + 1);
    body_->print(out, level + 1);
  }
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    std::string arrayReturn = array_->returnType(runner, stackFrame);
    if (arrayReturn.substr(0, 6) != "array-") {
      runner->errorAt(this, "Cannot iterate non-array type");
    }
    int arrayLength = std::stoi(
        arrayReturn.substr(arrayReturn.find('-') + 1,
                           arrayReturn.find(':') - arrayReturn.find('-') - 1));
    std::string returnType = arrayReturn.substr(arrayReturn.find(':') + 1);

    int size = sizeOfType(returnType);
    int allocs = size * arrayLength;
    void *value = runner->alloc(allocs);

    void *array, *array2;
    array_->run(runner, stackFrame, &array);
    array2 = array;  // there is a bug here
    RunnerStackFrame newStackFrame(stackFrame);

    ASTType *elementType =
        typeFromTypeRep(arrayReturn.substr(arrayReturn.find(':') + 1));
    void *forOut = newStackFrame.allocVariable(variable_, elementType, runner);
    for (int i = 0; i < arrayLength; i++) {
      std::memcpy(forOut,
                  (char *)array2 + i * sizeOfType(arrayReturn.substr(
                                           arrayReturn.find(":") + 1)),
                  sizeOfType(arrayReturn.substr(arrayReturn.find(":") + 1)));
      body_->run(runner, &newStackFrame, ((char *)value) + i * size);
    }

    *(void **)out = value;
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    std::string arrayReturn = array_->returnType(runner, stack);
    RunnerStackFrame newStackFrame(stack);
    ASTType *elementType =
        typeFromTypeRep(arrayReturn.substr(arrayReturn.find(':') + 1));
    newStackFrame.allocVariable(variable_, elementType, runner);
    std::string type =
        std::string("array-") +
        arrayReturn.substr(arrayReturn.find('-') + 1,
                           arrayReturn.find(':') - arrayReturn.find('-') - 1) +
        ":" + body_->returnType(runner, &newStackFrame);
    return type;
  }

 private:
  std::string variable_;
  ASTNode *array_;
  ASTNode *body_;
};
class ASTRange : public ASTNode {
 public:
  ASTRange(ASTNode *start, ASTNode *end, unsigned long pos = 0)
      : ASTNode(pos), start_(start), end_(end) {}
  virtual ~ASTRange() {
    delete start_;
    delete end_;
  }
  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Range: " << std::endl;
    start_->print(out, level + 1);
    end_->print(out, level + 1);
  }
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    long start, end;
    start_->run(runner, stackFrame, &start);
    end_->run(runner, stackFrame, &end);
    int length = end - start;
    void *value = runner->alloc(length * sizeof(long));
    for (int i = 0; i < length; i++) {
      *((long *)value + i) = start + i;
    }
    *(void **)out = value;
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    long start, end;
    start_->run(runner, stack, &start);
    end_->run(runner, stack, &end);
    int length = end - start;
    std::cout << "type: " << std::endl;
    std::string type = "array-" + std::to_string(length) + ":i64";
    return type;
  }

 private:
  ASTNode *start_;
  ASTNode *end_;
};

class Parser {
 public:
  Parser(std::istream &in) : in_(in) {}

  ASTNode *parse(bool standalone = true) {
    ASTNodeList *result = parseGlobalBody();
    if (in_.peek() != EOF) {
      throw std::runtime_error("Unexpected token : EOF expected");
    }
    if (standalone) {
      result->add(new ASTReturn(
          new ASTFunctionCall(new ASTVariable("main"), new ASTNodeList())));
    }
    return result;
  }

 private:
  void skipComment() {
    while (in_.peek() != '\n' && in_.peek() != EOF) {
      in_.get();
    }
  }
  void skipWhitespace() {
    while (isspace(in_.peek()) || in_.peek() == '#') {
      char c_ = in_.get();
      if (c_ == '#') {
        skipComment();
      }
    }
  }
  bool isValidIdentifierChar(char i, int pos = 0) {
    if (pos == 0) {
      return isalpha(i) || i == '_';
    } else {
      return isalnum(i) || i == '_';
    }
  }
  bool isValidOperatorChar(char i, int pos = 0,
                           bool shouldIncludeSemicolon = false) {
    return i == '+' || i == '-' || i == '*' || i == '/' || i == '%' ||
           i == '^' || i == '&' || i == '|' || i == '!' || i == '~' ||
           i == '<' || i == '>' || i == '=' || i == '?' ||
           (shouldIncludeSemicolon && (i == ';' || i == ':'));
  }
  std::string parseString(std::string input){
    std::string result;
    for (long unsigned int i = 0; i < input.length(); i++) {
      if (input[i] == '\\') {
        i++;
        if (input[i] == 'n') {
          result += '\n';
        } else if (input[i] == 't') {
          result += '\t';
        } else if (input[i] == '\\') {
          result += '\\';
        } else {
          throw std::runtime_error("Invalid escape sequence");
        }
      } else {
        result += input[i];
      }
    }
    return result;
  }

  std::string parseIdentifier() {
    skipWhitespace();
    std::string result;
    int _p = 0;
    while (isValidIdentifierChar(in_.peek(), _p)) {
      result += in_.get();
      _p++;
    }
    skipWhitespace();
    return result;
  }
  std::string parseOperator(bool shouldIncludeSemicolon = false) {
    skipWhitespace();
    std::string result;
    int _p = 0;
    while (isValidOperatorChar(in_.peek(), _p, shouldIncludeSemicolon)) {
      result += in_.get();
      _p++;
    }
    skipWhitespace();
    return result;
  }
  std::string readLine() {
    std::string result;
    while (in_.peek() != ';') {
      result += in_.get();
    }
    in_.get();
    return result;
  }

  ASTNode *parseVariableDecl() {
    ASTType *type = parseType();
    std::string name = parseIdentifier();
    skipWhitespace();
    ASTNode *value;
    if (in_.peek() == '=') {
      in_.get();
      skipWhitespace();
      value = parseExpression();
      skipWhitespace();
    } else {
      value = nullptr;
    }
    return new ASTVariableDecl(name, type, value, in_.tellg());
  }

  ASTType *parseType() {
    skipWhitespace();
    if (in_.peek() == EOF) {
      throw std::runtime_error("Unexpected EOF");
    }
    std::streampos oldpos = in_.tellg();
    std::string name = parseIdentifier();
    skipWhitespace();
    if (in_.peek() == '<') {
      in_.get();
      skipWhitespace();
      ASTType *type = parseType();
      skipWhitespace();
      if (in_.peek() != '>') {
        throw std::runtime_error("Expected >");
      }
      in_.get();
      skipWhitespace();
      return new ASTTemplate(type, name, in_.tellg());
    }
    std::streampos newpos = in_.tellg();
    in_.seekg(oldpos);

    if (in_.peek() == '*') {
      in_.get();
      return new ASTPointer(parseType(), in_.tellg());
    } else if (in_.peek() == '[') {
      in_.get();
      if (in_.peek() == ']') {
        in_.get();
        return new ASTArray(parseType(), new ASTNumber(0, in_.tellg()),
                            in_.tellg());
      }
      ASTNode *size = parseExpression();
      if (in_.get() != ']') {
        throw std::runtime_error("Expected ']'");
      }
      return new ASTArray(parseType(), size, in_.tellg());
    }
    in_.seekg(newpos);
    return new ASTBaseType(name, in_.tellg());
  }

  ASTNodeList *parseGlobalBody() {
    auto result = parseBody(false);
    return result;
  }

  ASTNodeList *parseBody(bool errorOnEOF = true) {
    ASTNodeList *result = new ASTNodeList(in_.tellg());
    skipWhitespace();
    while (in_.peek() != '}') {
      skipWhitespace();
      if (in_.peek() == EOF) {
        if (errorOnEOF) {
          throw std::runtime_error("Unexpected EOF");
        } else {
          break;
        }
      }
      skipWhitespace();

      std::streampos oldpos = in_.tellg();

      std::string line = "";
      while (in_.peek() != ';') {
        line += in_.get();
      }
      in_.seekg(oldpos);

      enum {
        VAR_DECLARATION = 1,
        VAR_ASSIGNMENT = 2,
        EXPRESSION = 3,
      } expressionType = EXPRESSION;

      try {
        delete parseType();
        skipWhitespace();
        std::string identifier = parseIdentifier();
        skipWhitespace();
        std::string nextOperator = parseOperator();
        if ((nextOperator == "=" || in_.peek() == ';') && identifier != "") {
          expressionType = VAR_DECLARATION;
        }
      } catch (std::runtime_error &e) {
      }

      in_.seekg(oldpos);

      if (expressionType == EXPRESSION) {
        try {
          parseIdentifier();
          skipWhitespace();
          std::string nextOperator = parseOperator();
          if (nextOperator == "=") {
            expressionType = VAR_ASSIGNMENT;
          }
        } catch (std::runtime_error &e) {
        }
      }

      in_.seekg(oldpos);

      skipWhitespace();
      if (expressionType == VAR_DECLARATION) {
        result->add(parseVariableDecl());
        if (in_.get() != ';') {
          throw std::runtime_error("Expected ';' after variable declaration");
        }
      } else if (expressionType == VAR_ASSIGNMENT) {
        std::string name = parseIdentifier();
        if (in_.get() != '=') {
          throw std::runtime_error("Expected '='");
        }
        skipWhitespace();
        ASTNode *value = parseExpression();
        if (in_.get() != ';') {
          throw std::runtime_error("Expected ';' after variable assignment");
        }
        result->add(new ASTVariableAssign(name, value, in_.tellg()));
      } else {
        ASTNode *value = parseExpression();
        result->add(value);
        if (in_.get() != ';') {
          throw std::runtime_error("Expected ';' after expression");
        }
      }

      skipWhitespace();
    }
    return result;
  }

  ASTFunctionArg *parseFunctionArg() {
    std::string name;
    int _p = 0;
    while (isValidIdentifierChar(in_.peek(), _p)) {
      name += in_.get();
      _p++;
    }
    skipWhitespace();
    if (in_.peek() != ':') {
      throw std::runtime_error("Expected ':'");
    }
    in_.get();
    skipWhitespace();
    ASTType *type = parseType();
    skipWhitespace();
    return new ASTFunctionArg(type, name, in_.tellg());
  }

  std::vector<ASTFunctionArg *> parseFunctionArgs() {
    std::vector<ASTFunctionArg *> result;
    while (in_.peek() != '{') {
      result.push_back(parseFunctionArg());
      skipWhitespace();

      if (in_.peek() == ',') {
        in_.get();
      } else {
      }
    }
    skipWhitespace();
    return result;
  }

  ASTNode *parseLambda() {
    if (in_.get() != '%') {
      throw std::runtime_error("Expected '%'");
    }
    skipWhitespace();
    std::vector<ASTFunctionArg *> args = parseFunctionArgs();
    if (in_.get() != '{') {
      throw std::runtime_error("Expected '{'");
    }
    ASTNodeList *body = parseBody();
    if (in_.get() != '}') {
      throw std::runtime_error("Expected '}'");
    }
    skipWhitespace();
    if (in_.peek() == '-') {
      std::string op = parseOperator();
      if (op == "->") {
        skipWhitespace();
        ASTType *type = parseType();
        return new ASTLambda(args, body, type, in_.tellg());
      }
    }
    body->add(new ASTReturn(new ASTNumber(0, in_.tellg()), in_.tellg()));
    return new ASTLambda(args, body, new ASTBaseType("void", in_.tellg()),
                         in_.tellg());
  }

  ASTNode *parseExpression() {
    skipWhitespace();
    ASTNode *result;
    skipWhitespace();
    if (in_.peek() == '%') {
      skipWhitespace();
      result = parseLambda();
      skipWhitespace();
    } else if (in_.peek() == '[') {
      in_.get();
      skipWhitespace();
      std::streampos oldpos = in_.tellg();
      ASTNode *expr = nullptr;
      if (in_.peek() != ']') {
        expr = parseExpression();
        skipWhitespace();
      }
      if (in_.peek() == ',' || in_.peek() == ']') {
        in_.seekg(oldpos);
        ASTNodeList *body = new ASTNodeList(in_.tellg());
        while (in_.peek() != ']') {
          skipWhitespace();
          body->add(parseExpression());
          skipWhitespace();
          if (in_.peek() == ',') {
            in_.get();
          } else {
            if (in_.peek() != ']') {
              throw std::runtime_error("Expected ',' or ']'");
            }
          }
          skipWhitespace();
        }
        in_.get();
        skipWhitespace();
        result = new ASTArrayLiteral(body, in_.tellg());
      } else {
        std::string op = parseIdentifier();
        if (op == "for") {
          skipWhitespace();
          std::string varName = parseIdentifier();
          skipWhitespace();
          if (in_.get() != ':') {
            throw std::runtime_error("Expected ':'");
          }
          skipWhitespace();
          ASTNode *array = parseExpression();
          skipWhitespace();
          if (in_.get() != ']') {
            throw std::runtime_error("Expected ']'");
          }
          skipWhitespace();
          result = new ASTArrayComprehension(varName, array, expr, in_.tellg());

        } else {
          throw std::runtime_error(
              op + " is not a valid array comprehension operator");
        }
      }
    } else if (isValidIdentifierChar(in_.peek())) {
      std::string name = parseIdentifier();

      skipWhitespace();
      // special identifiers
      if (name == "if") {
        ASTNode *condition = parseExpression();
        skipWhitespace();
        if (in_.get() != '{') {
          throw std::runtime_error("Expected '{'");
        }
        ASTNode *ifBody = parseBody();
        skipWhitespace();
        if (in_.get() != '}') {
          throw std::runtime_error("Expected '}'");
        }
        skipWhitespace();
        ASTNode *elseBody = nullptr;
        if (in_.peek() == 'e') {
          std::string elseName = parseIdentifier();
          if (elseName != "else") {
            throw std::runtime_error("Expected 'else'");
          }
          skipWhitespace();
          if (in_.get() != '{') {
            throw std::runtime_error("Expected '{'");
          }
          skipWhitespace();
          elseBody = parseBody();
          if (in_.get() != '}') {
            throw std::runtime_error("Expected '}'");
          }
          skipWhitespace();
        }
        result = new ASTIf(condition, ifBody, elseBody, in_.tellg());
      } else if (name == "while") {
        ASTNode *condition = parseExpression();
        skipWhitespace();
        if (in_.get() != '{') {
          throw std::runtime_error("Expected '{'");
        }
        skipWhitespace();
        ASTNode *body = parseBody();
        if (in_.get() != '}') {
          throw std::runtime_error("Expected '}'");
        }
        skipWhitespace();
        result = new ASTWhile(condition, body, in_.tellg());
      } else if (name == "for") {
        skipWhitespace();
        std::string varName = parseIdentifier();
        skipWhitespace();
        if (in_.get() != ':') {
          throw std::runtime_error("Expected ':'");
        }
        skipWhitespace();
        ASTNode *array = parseExpression();
        skipWhitespace();
        if (in_.get() != '{') {
          throw std::runtime_error("Expected '{'");
        }
        skipWhitespace();
        ASTNode *body = parseBody();
        if (in_.get() != '}') {
          throw std::runtime_error("Expected '}'");
        }
        skipWhitespace();
        result = new ASTForIn(varName, array, body, in_.tellg());
      } else if (name == "return" || name == "ret") {
        skipWhitespace();
        if (in_.peek() == ';') {
          return new ASTReturn(nullptr, in_.tellg());
        } else {
          ASTNode *value = parseExpression();
          result = new ASTReturn(value, in_.tellg());
        }
        skipWhitespace();
      } else if (name == "ref") {
        skipWhitespace();
        std::string value = parseIdentifier();
        result = new ASTRef(value, in_.tellg());
      } else if (name == "deref") {
        ASTNode *value = parseExpression();
        result = new ASTDeref(value, in_.tellg());
      } else if (name == "null") {
        result = new ASTNull(in_.tellg());
      } else if (name == "true") {
        result = new ASTBool(true, in_.tellg());
      } else if (name == "false") {
        result = new ASTBool(false, in_.tellg());
      } else {
        result = new ASTVariable(name, in_.tellg());
        skipWhitespace();
      }
    } else {
      result = parseTerm();
    }
    skipWhitespace();
    if (in_.peek() == '(') {
      in_.get();
      ASTNodeList *args = new ASTNodeList(in_.tellg());
      while (in_.peek() != ')') {
        skipWhitespace();
        args->add(parseExpression());
        skipWhitespace();
        if (in_.peek() == ',') {
          in_.get();
        } else {
        }
      }
      in_.get();

      result = new ASTFunctionCall(result, args, in_.tellg());
    }
    skipWhitespace();
    while (in_.peek() == '[') {
      in_.get();
      ASTNode *index = parseExpression();
      skipWhitespace();
      if (in_.get() != ']') {
        throw std::runtime_error("Expected ']'");
      }
      skipWhitespace();
      result = new ASTArrayAccess(result, index, in_.tellg());
    }
    while (isValidOperatorChar(in_.peek()) || in_.peek() == '.') {
      std::streampos oldpos = in_.tellg();
      std::string op = parseOperator();
      if (in_.peek() == '.') {
        in_.get();
        if (in_.peek() == '.') {
          in_.get();
          op = "..";
          skipWhitespace();
        } else {
          in_.seekg(oldpos);
          break;
        }
      }
      if (op == "=") {
        in_.seekg(oldpos);
        break;
      } else if (op == "->") {
        skipWhitespace();
        ASTType *type = parseType();
        result = new ASTCast(result, type, in_.tellg());
      } else if (op == "..") {
        skipWhitespace();
        ASTNode *end = parseExpression();
        result = new ASTRange(result, end, in_.tellg());
      } else {
        skipWhitespace();
        auto right = parseExpression();
        result = new ASTBinaryOp(op, result, right, in_.tellg());
      }
    }
    return result;
  }

  ASTNode *parseTerm() {
    auto result = parseFactor();
    return result;
  }

  ASTNode *parseFactor() {
    if (in_.peek() == '(') {
      in_.get();
      auto result = parseExpression();
      if (in_.get() != ')') {
        throw std::runtime_error("Expected ')'");
      }
      return result;
    } else if (isdigit(in_.peek())) {
      double value = 0;
      unsigned char isFloat = 0;
      while (isdigit(in_.peek()) || in_.peek() == '.') {
        if (in_.peek() == '.' && !isFloat) {
          std::streampos oldpos = in_.tellg();
          in_.get();
          if (in_.peek() == '.') {
            in_.seekg(oldpos);
            break;
          }
          isFloat = 1;
        } else {
          if (isFloat) {
            value += (in_.get() - '0') / std::pow(10.0, isFloat);
            isFloat++;
          } else {
            value *= 10;
            value += in_.get() - '0';
          }
        }
      }
      if (isFloat) {
        return new ASTDouble(value, in_.tellg());
      } else {
        return new ASTNumber(value, in_.tellg());
      }
    } else if (in_.peek() == '"') {
      in_.get();
      std::string value;
      while (in_.peek() != '"') {
        value += in_.get();
      }
      in_.get();
      return new ASTString(parseString(value), in_.tellg());
    } else if (in_.peek() == '\'') {
      // char
      in_.get();
      std::string value;
      while (in_.peek() != '\'') {
        value += in_.get();
      }
      in_.get();
      std::string str = parseString(value);
      if (str.size() != 1) {
        throw std::runtime_error("Character literal must be a single character");
      }
      return new ASTChar(str[0], in_.tellg());
    } else {
      throw std::runtime_error("Unexpected token: ");
    }
  }

  std::istream &in_;
};

int main() {
  std::cout.precision(std::numeric_limits<double>::max_digits10);

  std::ifstream file("tests/initial.wha");
  std::istream &code = static_cast<std::istream &>(file);

  Parser parser(code);

  ASTNode *ast;

  try {
    ast = parser.parse();
  } catch (std::runtime_error &e) {
    errorWithMessage(e.what(), code, (int)code.tellg());
  }

  ast->print(std::cout);

  int exitCode = 0;

  std::string Error;
  //TODO: Parse from command line args
  auto triple_name_str = llvm::sys::getDefaultTargetTriple();
  auto CPU = "generic";
 
    auto TargetTriple = llvm::sys::getDefaultTargetTriple();
  Module->setTargetTriple(TargetTriple);
  auto Target = llvm::TargetRegistry::lookupTarget(TargetTriple, Error);
  llvm::TargetOptions opt;
  auto RM = llvm::Optional<llvm::Reloc::Model>();
  auto Features = "";

  auto TheTargetMachine =
      Target->createTargetMachine(TargetTriple, CPU, Features, opt, RM);
  
  auto DL = llvm::DataLayout(&*Module);
  
  llvm::Function::Create(llvm::FunctionType::get(TheBuilder->getVoidTy()->getPointerTo(), std::vector<llvm::Type *>({TheBuilder->getVoidTy()->getPointerTo(), TheBuilder->getIntPtrTy(DL), TheBuilder->getInt32Ty(), TheBuilder->getInt32Ty(),TheBuilder->getInt32Ty(), TheBuilder->getInt64Ty()}), false), llvm::GlobalObject::ExternalLinkage, "mmap", *Module);

  
  auto entry = llvm::Function::Create(llvm::FunctionType::get(llvm::Type::getVoidTy(*TheContext), std::vector<llvm::Type *>(), false), llvm::GlobalValue::ExternalLinkage, "main", *Module);
  auto program = llvm::BasicBlock::Create(*TheContext, "", entry);
  TheBuilder->SetInsertPoint(program);
  CompilerStackFrame frame;
  ast->codegen(&frame);
  llvm::InitializeAllTargetInfos();
  llvm::InitializeAllTargets();
  llvm::InitializeAllTargetMCs();
  llvm::InitializeAllAsmParsers();
  llvm::InitializeAllAsmPrinters();

  
  llvm::verifyModule(*Module, &llvm::errs());
  // OPTIMIZATION PASSES
  // pre-opt print
  Module->print(llvm::errs(), nullptr);

  llvm::PassBuilder pb;
  llvm::LoopAnalysisManager lam;
  llvm::FunctionAnalysisManager fam;
  llvm::CGSCCAnalysisManager cgsccam;
  llvm::ModuleAnalysisManager mam;

  pb.registerModuleAnalyses(mam);
  pb.registerCGSCCAnalyses(cgsccam);
  pb.registerFunctionAnalyses(fam);
  pb.registerLoopAnalyses(lam);

  pb.crossRegisterProxies(lam, fam, cgsccam, mam);

  llvm::ModulePassManager module_pass_manager =
      pb.buildPerModuleDefaultPipeline(
          llvm::OptimizationLevel::O2);

  module_pass_manager.run(*Module, mam);
  // re-verify post optimization
  llvm::verifyModule(*Module, &llvm::errs());
  // TODO: get output file name
  std::error_code EC;
  auto output_file = "out.o";
  llvm::raw_fd_ostream dest(output_file, EC, llvm::sys::fs::OF_None);

  if (EC) {
    llvm::errs() << "Could not open file: " << EC.message();
    return 1;
  }

  llvm::legacy::PassManager pass;
  auto FileType = llvm::CGFT_ObjectFile;
  // check module
  Module->print(llvm::errs(), nullptr);

  if (TheTargetMachine->addPassesToEmitFile(pass, dest, nullptr, FileType)) {
    llvm::errs() << "TheTargetMachine can't emit a file of this type";
    return 1;
  }

  pass.run(*Module);
  dest.flush();
  llvm::outs() << "Wrote output to " << output_file << "\n";

  Runner runner(ast, code);
  /*
  runner.generateFunction("print",
                          new ASTNodeList({new ASTFunctionArg(
                              new ASTPointer(new ASTBaseType("char")), "str")}),
                          [](Runner *runner, RunnerStackFrame *stackFrame) {
                            char *str = stackFrame->getVariable<char *>("str");

                            std::cout << str << std::endl;
                          });
  runner.generateFunction(
      "printintl",
      new ASTNodeList(
          {new ASTFunctionArg(new ASTPointer(new ASTBaseType("char")), "str"),
           new ASTFunctionArg(new ASTBaseType("i64"), "number")}),
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        char *str = stackFrame->getVariable<char *>("str");
        int number = stackFrame->getVariable<int>("number");

        std::cout << str << number << std::endl;
      });
  runner.generateFunction(
      "printint",
      new ASTNodeList({new ASTFunctionArg(new ASTBaseType("i64"), "number")}),
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        int number = stackFrame->getVariable<int>("number");

        std::cout << number << std::endl;
      });
  runner.generateFunction(
      "printdouble",
      new ASTNodeList(
          {new ASTFunctionArg(new ASTBaseType("double"), "number")}),
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        double number = stackFrame->getVariable<double>("number");

        std::cout << number << std::endl;
      });

  runner.run(&exitCode);

  return exitCode;
  */
  return 0;
}