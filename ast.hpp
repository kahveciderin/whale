#pragma once

#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Value.h>
#include <map>
#include <vector>

extern std::unique_ptr<llvm::LLVMContext> TheContext;
extern std::unique_ptr<llvm::IRBuilder<>> TheBuilder;
extern std::unique_ptr<llvm::Module> Module;

class CompilerStackFrame {
  public:
    CompilerStackFrame(CompilerStackFrame *parent = nullptr);
    void set(std::string name, llvm::Value *varspace);
    virtual llvm::Value *resolve(std::string name);
  protected:
  CompilerStackFrame *parent;
  std::map<std::string, llvm::Value *> data;
};

class LambdaStackFrame : public CompilerStackFrame {
  public:
    LambdaStackFrame(CompilerStackFrame *upper, std::function<llvm::Value *(std::string, llvm::Value *)> register_implicit);
    virtual llvm::Value *resolve(std::string name);
  private:
  std::function<llvm::Value *(std::string name, llvm::Value *actual)> implicit_handler;
  protected:
  CompilerStackFrame *parent;
};

class WithPos {
  public:
  WithPos(unsigned long p);
  unsigned long pos_;
};

class ASTNodeList;
class Runner;
class RunnerStackFrame;

class ASTNode : public WithPos {
 public:
  ASTNode(unsigned long pos);
  virtual ~ASTNode();
  virtual void print(std::ostream &out, int level = 0) const = 0;
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out = nullptr) const = 0;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const = 0;

  virtual llvm::Value *codegen(CompilerStackFrame *frame) = 0;
};
class ASTType : public WithPos {
  public:
    virtual ~ASTType();

    ASTType(unsigned long pos);

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
  ASTBaseType(const std::string &name, unsigned long pos = 0);
  virtual ~ASTBaseType();
  virtual void print(std::ostream &out, int level) const;
  virtual llvm::Type *into_llvm_type();
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                void *out) const;
  virtual const std::string returnType(Runner *runner,
                RunnerStackFrame *stack) const;
 private:
  std::string name_;
};
class ASTFunctionType : public ASTType {
  public:
    ASTFunctionType(ASTType *return_type, std::vector<ASTType *> arg_types, unsigned long pos = 0);
    virtual ~ASTFunctionType();
    virtual void print(std::ostream &out, int level) const;
    virtual llvm::Type *into_llvm_type();
    virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                void *out) const;
    virtual const std::string returnType(Runner *runner,
                RunnerStackFrame *stack) const;
  private:
    ASTType *ret_type_;
    std::vector<ASTType *> arg_types_;
};
class ASTFunctionArg;
class ASTNodeList : public ASTNode {
 public:
  ASTNodeList(const std::vector<ASTNode *> &nodes, unsigned long pos = 0);
  ASTNodeList(unsigned long pos = 0);

  virtual ~ASTNodeList();

  void add(ASTNode *node);

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

  std::vector<ASTNode *> nodes_;

 private:
};
class ASTArrayLiteral : public ASTNode {
 public:
  ASTArrayLiteral(ASTNodeList *list, unsigned long pos = 0);
  virtual void print(std::ostream &out, int level) const;
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  ASTNodeList *list_;
};

class ASTTemplate : public ASTType {
 public:
  ASTTemplate(ASTType *type, const std::string &name, unsigned long pos = 0);
  virtual ~ASTTemplate();
  virtual llvm::Type *into_llvm_type();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

 private:
  ASTType *type_;
  std::string name_;
};

class ASTFunctionArg : public WithPos {
 public:
  ASTFunctionArg(ASTType *type, const std::string &name, unsigned long pos);
  
  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  ASTType *type_;
  std::string name_;
};

class ASTLambda : public ASTNode {
 public:
  ASTLambda(std::vector<ASTFunctionArg *> args, ASTNode *body, ASTType *type, unsigned long pos = 0);

  virtual ~ASTLambda();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

  std::vector<ASTFunctionArg *> args_;
  ASTNode *body_;
  ASTType *type_;

 private:
};

class ASTNativeFunction : public ASTNode {
 public:
  ASTNativeFunction(void (*function)(Runner *, RunnerStackFrame *),
                    unsigned long pos = 0);

  virtual ~ASTNativeFunction();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  void (*function_)(Runner *, RunnerStackFrame *);
};

void errorWithMessage(const std::string &msg, std::istream &code_,
                      unsigned long pos);



class ASTPointer : public ASTType {
 public:
  ASTPointer(ASTType *type, unsigned long pos = 0);

  virtual ~ASTPointer();

  virtual llvm::Type *into_llvm_type();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

 private:
  ASTType *type_;
};
class ASTArray : public ASTType {
 public:
  ASTArray(ASTType *type, ASTNode *size, unsigned long pos = 0);

  virtual ~ASTArray();

  virtual llvm::Type *into_llvm_type();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  ASTType *type_;

 private:
  ASTNode *size_;
};

class ASTNumber : public ASTNode {
 public:
  ASTNumber(long int value, unsigned long pos = 0);

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  long int value_;
};
class ASTChar : public ASTNode {
 public:
  ASTChar(char value, unsigned long pos = 0);
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

virtual void print(std::ostream &out, int level) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);
 private:
  char value_;
};
class ASTDouble : public ASTNode {
 public:
  ASTDouble(double value, unsigned long pos = 0);

  virtual void print(std::ostream &out, int level) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  double value_;
};
class ASTString : public ASTNode {
 public:
  ASTString(const std::string &value, unsigned long pos = 0);

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  std::string value_;
};

class ASTBinaryOp : public ASTNode {
 public:
  ASTBinaryOp(std::string op, ASTNode *left, ASTNode *right,
              unsigned long pos = 0);

  virtual ~ASTBinaryOp();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

 private:
  std::string op_;
  ASTNode *left_;
  ASTNode *right_;
};

class ASTCast : public ASTNode {
 public:
  ASTCast(ASTNode *value, ASTType *type, unsigned long pos = 0);

  virtual ~ASTCast();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  ASTNode *value_;
  ASTType *type_;
};

class ASTVariableDecl : public ASTNode {
 public:
  ASTVariableDecl(const std::string &name, ASTType *type, ASTNode *value,
                  unsigned long pos = 0);

  virtual ~ASTVariableDecl();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  std::string name_;
  ASTType *type_;
  ASTNode *value_;
};
class ASTVariable : public ASTNode {
 public:
  ASTVariable(const std::string &name, unsigned long pos = 0);

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  std::string name_;
};
class ASTVariableAssign : public ASTNode {
 public:
  ASTVariableAssign(const std::string &name, ASTNode *value,
                    unsigned long pos = 0);

  virtual ~ASTVariableAssign();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  std::string name_;
  ASTNode *value_;
};
class ASTFunctionCall : public ASTNode {
 public:
  ASTFunctionCall(ASTNode *pointer, ASTNodeList *args, unsigned long pos = 0);

  virtual ~ASTFunctionCall();

  virtual void print(std::ostream &out, int level) const;

  ASTLambda *getLambda(Runner *runner, RunnerStackFrame *stackFrame) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  ASTNode *pointer_;
  ASTNodeList *args_;
};
class ASTIf : public ASTNode {
 public:
  ASTIf(ASTNode *condition, ASTNode *body, ASTNode *elseBody,
        unsigned long pos = 0);

  virtual ~ASTIf();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  ASTNode *condition_;
  ASTNode *body_;
  ASTNode *elseBody_;
};
class ASTWhile : public ASTNode {
 public:
  ASTWhile(ASTNode *condition, ASTNode *body, unsigned long pos = 0);

  virtual ~ASTWhile();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

 private:
  ASTNode *condition_;
  ASTNode *body_;
};
class ASTReturn : public ASTNode {
 public:
  ASTReturn(ASTNode *value, unsigned long pos = 0);

  virtual ~ASTReturn();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  ASTNode *value_;
};
class ASTRef : public ASTNode {
 public:
  ASTRef(std::string &variable, unsigned long pos = 0);

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  std::string variable_;
};
class ASTDeref : public ASTNode {
 public:
  ASTDeref(ASTNode *value, unsigned long pos = 0);

  virtual ~ASTDeref();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  ASTNode *value_;
};
class ASTNull : public ASTNode {
 public:
  ASTNull(unsigned long pos = 0);

  virtual ~ASTNull();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
};
class ASTBool : public ASTNode {
 public:
  ASTBool(bool value, unsigned long pos = 0);

  virtual ~ASTBool();

  virtual void print(std::ostream &out, int level) const;

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  bool value_;
};
class ASTArrayAccess : public ASTNode {
 public:
  ASTArrayAccess(ASTNode *array, ASTNode *index, unsigned long pos = 0);
  virtual ~ASTArrayAccess();
  virtual void print(std::ostream &out, int level) const;
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  ASTNode *array_;
  ASTNode *index_;
};



class ASTForIn : public ASTNode {
 public:
  ASTForIn(std::string &variable, ASTNode *array, ASTNode *body,
           unsigned long pos = 0);
  virtual ~ASTForIn();
  virtual void print(std::ostream &out, int level) const;
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);

 private:
  std::string variable_;
  ASTNode *array_;
  ASTNode *body_;
};
class ASTArrayComprehension : public ASTNode {
 public:
  ASTArrayComprehension(std::string &variable, ASTNode *array, ASTNode *body,
                        unsigned long pos = 0);
  virtual ~ASTArrayComprehension();
  virtual void print(std::ostream &out, int level) const;
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;
  virtual llvm::Value *codegen(CompilerStackFrame *frame);
 private:
  std::string variable_;
  ASTNode *array_;
  ASTNode *body_;
};
class ASTRange : public ASTNode {
 public:
  ASTRange(ASTNode *start, ASTNode *end, unsigned long pos = 0);
  virtual ~ASTRange();
  virtual void print(std::ostream &out, int level) const;
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const;
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const;

  virtual llvm::Value *codegen(CompilerStackFrame *frame);
 private:
  ASTNode *start_;
  ASTNode *end_;
};