#include <llvm/ADT/APFloat.h>
#include <llvm/ADT/APInt.h>
#include <llvm/ADT/Sequence.h>
#include <llvm/Analysis/CGSCCPassManager.h>
#include <llvm/Analysis/LoopAnalysisManager.h>
#include <llvm/IR/Attributes.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/DataLayout.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalObject.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/PassManager.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/Verifier.h>
#include <llvm/MC/TargetRegistry.h>
#include <llvm/Passes/PassBuilder.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/CodeGen.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/Host.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Target/TargetLoweringObjectFile.h>
#include <llvm/Target/TargetMachine.h>
#include <llvm/Target/TargetOptions.h>
#include <sys/mman.h>
#include <sys/types.h>

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ast.hpp"
#include "parser.hpp"
#include "runner.hpp"

#define USE_LLVM 0

std::unique_ptr<llvm::LLVMContext> TheContext;
std::unique_ptr<llvm::IRBuilder<>> TheBuilder;
std::unique_ptr<llvm::Module> Module;

void init_module() {
  TheContext = std::make_unique<llvm::LLVMContext>();
  Module = std::make_unique<llvm::Module>("whale", *TheContext);
  TheBuilder = std::make_unique<llvm::IRBuilder<>>(*TheContext);
}

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

#if USE_LLVM
  std::string Error;
  // TODO: Parse from command line args
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

  llvm::Function::Create(
      llvm::FunctionType::get(
          TheBuilder->getVoidTy()->getPointerTo(),
          std::vector<llvm::Type *>(
              {TheBuilder->getVoidTy()->getPointerTo(),
               TheBuilder->getIntPtrTy(DL), TheBuilder->getInt32Ty(),
               TheBuilder->getInt32Ty(), TheBuilder->getInt32Ty(),
               TheBuilder->getInt64Ty()}),
          false),
      llvm::GlobalObject::ExternalLinkage, "mmap", *Module);

  auto entry = llvm::Function::Create(
      llvm::FunctionType::get(llvm::Type::getVoidTy(*TheContext),
                              std::vector<llvm::Type *>(), false),
      llvm::GlobalValue::ExternalLinkage, "main", *Module);
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
      pb.buildPerModuleDefaultPipeline(llvm::OptimizationLevel::O2);

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

#else
  int exitCode = 0;

  Runner runner(ast, code);
  runner.generateFunction(
      "print",
      {new ASTFunctionArg(new ASTPointer(new ASTBaseType("char")), "str")},
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        char *str = stackFrame->getVariable<char *>("str");

        std::cout << str << std::endl;
      });
  runner.generateFunction(
      "printintl",
      {new ASTFunctionArg(new ASTPointer(new ASTBaseType("char")), "str"),
       new ASTFunctionArg(new ASTBaseType("i64"), "number")},
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        char *str = stackFrame->getVariable<char *>("str");
        int number = stackFrame->getVariable<int>("number");

        std::cout << str << number << std::endl;
      });
  runner.generateFunction(
      "printint", {new ASTFunctionArg(
        new ASTBaseType("i64"),
        "number")},
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        int number = stackFrame->getVariable<int>("number");

        std::cout << number << std::endl;
      });
  runner.generateFunction(
      "printdouble", {new ASTFunctionArg(
        new ASTBaseType("double"),
        "number")},
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        double number = stackFrame->getVariable<double>("number");

        std::cout << number << std::endl;
      });

  runner.run(&exitCode);

  return exitCode;

#endif
  return 0;
}