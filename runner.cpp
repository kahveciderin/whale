#include "ast.hpp"
#include "parser.hpp"
#include "runner.hpp"
#include <iostream>

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

RunnerStackFrame::RunnerStackFrame(RunnerStackFrame *parent) : parent_(parent) {};
RunnerStackFrame::~RunnerStackFrame() {
    for (auto &kv : variables_) {
        free(kv.second);
    }
}

RunnerStackFrame::Variable::Variable(void *ptr, ASTType *type) : ptr_(ptr), type_(type) {};

RunnerStackFrame::Variable *RunnerStackFrame::getVariable(const std::string &name) {
    if (this->variables_[name] != nullptr) return this->variables_[name];
    if (this->parent_ != nullptr) return this->parent_->getVariable(name);
    throw std::runtime_error("Variable not found: " + name);
};
template <typename T>
T RunnerStackFrame::getVariable(const std::string &name) {
    return *(T *)this->getVariable(name)->ptr_;
};
void * RunnerStackFrame::allocVariable(const std::string &name, ASTType *type, Runner *runner) {
    int size;
    type->run(runner, this, &size);
    void *value = runner->alloc(size);
    this->variables_[name] = new Variable(value, type);
    return value;
};

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
void Runner::gc() {

};
void Runner::errorAt(const ASTNode *node, std::string message) {
  errorWithMessage(message, code_, node->pos_);
}

void ASTBaseType::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  *(int *)out = sizeOfType(name_);
}

const std::string ASTBaseType::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return name_;
}

void ASTFunctionType::run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
  *(int *)out = sizeOfType("fun");
}
const std::string ASTFunctionType::returnType(Runner *runner, RunnerStackFrame *stack) const {
  std::string args;
  for (auto arg : arg_types_) {
    args += ",";
    args += arg->returnType(runner, stack);
  }
  return std::string() + "fun(" + ret_type_->returnType(runner, stack) + args + ")" + std::string();
}

void ASTNodeList::run(Runner *runner, RunnerStackFrame *stackFrame,
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

const std::string ASTNodeList::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "void";
}

void ASTArrayLiteral::run(Runner *runner, RunnerStackFrame *stackFrame,
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
const std::string ASTArrayLiteral::returnType(Runner *runner,
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

void ASTTemplate::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  *(int *)out = sizeof(void *);  // templates always have a pointer size
}

const std::string ASTTemplate::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "template-" + name_ + ":" + type_->returnType(runner, stack);
}

void ASTFunctionArg::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  void *ptr = stackFrame->allocVariable(name_, (ASTType *)type_, runner);
  *(void **)out = ptr;
}

const std::string ASTFunctionArg::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return type_->returnType(runner, stack);
}

void ASTLambda::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  *(const ASTLambda **)out = this;
}

const std::string ASTLambda::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "fun";
}

void ASTNativeFunction::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  function_(runner, stackFrame);
}

const std::string ASTNativeFunction::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "void";
}

void ASTPointer::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  *(int *)out = sizeof(void *);
}

const std::string ASTPointer::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "pointer:" + type_->returnType(runner, stack);
}

void ASTArray::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  int sizeOutput, typeOutput;
  size_->run(runner, stackFrame, &sizeOutput);
  type_->run(runner, stackFrame, &typeOutput);
  *(int *)out = sizeOutput * typeOutput;
}

const std::string ASTArray::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  int sizeOutput;
  size_->run(runner, stack, &sizeOutput);
  return "array-" + std::to_string(sizeOutput) + ":" +
          type_->returnType(runner, stack);
}

void ASTNumber::run(Runner *runner, RunnerStackFrame *stackFrame,
                void *out) const {
  *(long int *)out = value_;
}

const std::string ASTNumber::returnType(Runner *runner,
                                    RunnerStackFrame *stack) const {
  return "i64";
}

void ASTChar::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  *(char *)out = value_;
}
const std::string ASTChar::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "char";
}

const std::string ASTDouble::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "double";
}

void ASTDouble::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  *(double *)out = value_;
}

void ASTString::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  *(const char **)out = value_.c_str();
}

const std::string ASTString::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "pointer:char";
}

void ASTBinaryOp::run(Runner *runner, RunnerStackFrame *stackFrame,
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

const std::string ASTBinaryOp::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return op_ == "/" ? "double" : "i64";
}

void ASTCast::run(Runner *runner, RunnerStackFrame *stackFrame,
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

const std::string ASTCast::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return type_->returnType(runner, stack);
}

void ASTVariableDecl::run(Runner *runner, RunnerStackFrame *stackFrame,
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

const std::string ASTVariableDecl::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "void";
}

void ASTVariable::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  auto it = stackFrame->getVariable(name_);

  if (out != nullptr) {
    *(unsigned long *)out = (unsigned long)*(unsigned long int **)(it->ptr_);
  }
}

const std::string ASTVariable::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return stack->getVariable(name_)->type_->returnType(runner, stack);
}

void ASTVariableAssign::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  auto it = stackFrame->getVariable(name_)->ptr_;
  value_->run(runner, stackFrame, it);
}

const std::string ASTVariableAssign::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "void";
}

ASTLambda *ASTFunctionCall::getLambda(Runner *runner, RunnerStackFrame *stackFrame) const {
  void *lambdaAddress;
  pointer_->run(runner, stackFrame, &lambdaAddress);

  ASTLambda *lambda = (ASTLambda *)lambdaAddress;
  return lambda;
}

void ASTFunctionCall::run(Runner *runner, RunnerStackFrame *stackFrame,
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

const std::string ASTFunctionCall::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return getLambda(runner, stack)->type_->returnType(runner, stack);
}

void ASTIf::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  int conditionOutput;
  condition_->run(runner, stackFrame, &conditionOutput);
  if (conditionOutput) {
    body_->run(runner, stackFrame, out);
  } else if (elseBody_ != nullptr) {
    elseBody_->run(runner, stackFrame, out);
  }
}

const std::string ASTIf::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "void";
}

void ASTWhile::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  int conditionOutput;
  condition_->run(runner, stackFrame, &conditionOutput);
  while (conditionOutput) {
    body_->run(runner, stackFrame, out);
    condition_->run(runner, stackFrame, &conditionOutput);
  }
}

const std::string ASTWhile::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "void";
}

void ASTReturn::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  if (value_) {
    value_->run(runner, stackFrame, out);
  }
  runner->_return = true;
}
const std::string ASTReturn::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "void";
}

void ASTRef::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  void *variableAddress = stackFrame->getVariable(variable_)->ptr_;
  if (out != nullptr) {
    *(unsigned long **)out = (unsigned long *)variableAddress;
  }
}
const std::string ASTRef::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "pointer:" +
          stack->getVariable(variable_)->type_->returnType(runner, stack);
}

void ASTDeref::run(Runner *runner, RunnerStackFrame *stackFrame,
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

const std::string ASTDeref::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  if (value_->returnType(runner, stack).substr(0, 8) == "pointer:") {
    return value_->returnType(runner, stack).substr(8);
  } else {
    runner->errorAt(this, "Cannot dereference non-pointer type");
    return "void";
  }
}

void ASTNull::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  if (out != nullptr) {
    *(unsigned long *)out = 0;
  }
}

const std::string ASTNull::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "pointer:void";
}

void ASTBool::run(Runner *runner, RunnerStackFrame *stackFrame,
                  void *out) const {
  if (out != nullptr) {
    *(bool *)out = value_;
  }
}

const std::string ASTBool::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "bool";
}


void ASTArrayAccess::run(Runner *runner, RunnerStackFrame *stackFrame,
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

const std::string ASTArrayAccess::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  if (array_->returnType(runner, stack).substr(0, 6) != "array-" &&
      array_->returnType(runner, stack).substr(0, 7) != "pointer") {
    runner->errorAt(this, "Cannot access non-accessible type");
  }

  return array_->returnType(runner, stack)
      .substr(array_->returnType(runner, stack).find(':') + 1);
}

void ASTForIn::run(Runner *runner, RunnerStackFrame *stackFrame,
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

const std::string ASTForIn::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  return "void";
}

void ASTArrayComprehension::run(Runner *runner, RunnerStackFrame *stackFrame,
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
const std::string ASTArrayComprehension::returnType(Runner *runner,
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

void ASTRange::run(Runner *runner, RunnerStackFrame *stackFrame,
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

const std::string ASTRange::returnType(Runner *runner,
                                      RunnerStackFrame *stack) const {
  long start, end;
  start_->run(runner, stack, &start);
  end_->run(runner, stack, &end);
  int length = end - start;
  std::cout << "type: " << std::endl;
  std::string type = "array-" + std::to_string(length) + ":i64";
  return type;
}