#pragma once
#include "ast.hpp"

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
  RunnerStackFrame(RunnerStackFrame *parent = nullptr);
  ~RunnerStackFrame();

  class Variable {
   public:
    Variable(void *ptr, ASTType *type);
    void *ptr_;
    ASTType *type_;
  };

  Variable *getVariable(const std::string &name);
  template <typename T>
  T getVariable(const std::string &name);
  void *allocVariable(const std::string &name, ASTType *type, Runner *runner);

  RunnerStackFrame *parent_;
  std::map<std::string, Variable *> variables_;
};
