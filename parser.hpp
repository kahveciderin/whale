#pragma once
#include <string>
#include "ast.hpp"
class Parser {
 public:
  Parser(std::istream &in);

  ASTNode *parse(bool standalone = true);

 private:
  void skipComment();
  void skipWhitespace();
  bool isValidIdentifierChar(char i, int pos = 0);
  bool isValidOperatorChar(char i, int pos = 0, bool shouldIncludeSemicolon = false);
  std::string parseString(std::string input);

  std::string parseIdentifier();
  std::string parseOperator(bool shouldIncludeSemicolon = false);
  std::string readLine();

  ASTNode *parseVariableDecl();

  ASTType *parseType();

  ASTNodeList *parseGlobalBody();

  ASTNodeList *parseBody(bool errorOnEOF = false);
  
  ASTFunctionArg *parseFunctionArg();

  std::vector<ASTFunctionArg *> parseFunctionArgs();

  ASTNode *parseLambda();

  ASTNode *parseExpression();

  ASTNode *parseTerm();

  ASTNode *parseFactor();

  std::istream &in_;
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
ValueType parseType(std::string type);
size_t sizeOfType(std::string type);
ASTType *typeFromTypeRep(std::string type);