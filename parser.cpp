#include "parser.hpp"
#include <iostream>
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

Parser::Parser(std::istream &in) : in_(in) {}

ASTNode *Parser::parse(bool standalone) {
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
void Parser::skipComment() {
while (in_.peek() != '\n' && in_.peek() != EOF) {
    in_.get();
}
}
void Parser::skipWhitespace() {
while (isspace(in_.peek()) || in_.peek() == '#') {
    char c_ = in_.get();
    if (c_ == '#') {
    skipComment();
    }
}
}
bool Parser::isValidIdentifierChar(char i, int pos) {
if (pos == 0) {
    return isalpha(i) || i == '_';
} else {
    return isalnum(i) || i == '_';
}
}
bool Parser::isValidOperatorChar(char i, int pos,
                        bool shouldIncludeSemicolon) {
return i == '+' || i == '-' || i == '*' || i == '/' || i == '%' ||
        i == '^' || i == '&' || i == '|' || i == '!' || i == '~' ||
        i == '<' || i == '>' || i == '=' || i == '?' ||
        (shouldIncludeSemicolon && (i == ';' || i == ':'));
}
std::string Parser::parseString(std::string input){
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

std::string Parser::parseIdentifier() {
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
std::string Parser::parseOperator(bool shouldIncludeSemicolon) {
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
std::string Parser::readLine() {
std::string result;
while (in_.peek() != ';') {
    result += in_.get();
}
in_.get();
return result;
}

ASTNode *Parser::parseVariableDecl() {
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

ASTType *Parser::parseType() {
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

ASTNodeList *Parser::parseGlobalBody() {
auto result = parseBody(false);
return result;
}

ASTNodeList *Parser::parseBody(bool errorOnEOF) {
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

ASTFunctionArg *Parser::parseFunctionArg() {
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

std::vector<ASTFunctionArg *> Parser::parseFunctionArgs() {
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

ASTNode *Parser::parseLambda() {
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

ASTNode *Parser::parseExpression() {
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

ASTNode *Parser::parseTerm() {
auto result = parseFactor();
return result;
}

ASTNode *Parser::parseFactor() {
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