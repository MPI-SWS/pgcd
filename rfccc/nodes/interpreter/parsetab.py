
# parsetab.py
# This file is automatically generated. Do not edit.
# pylint: disable=W,C,R
_tabversion = '3.10'

_lr_method = 'LALR'

_lr_signature = 'leftANDORLTLEGTGEEQNErightNOTleftPLUSMINUSleftTIMESDIVIDEMODrightUMINUSSINTANCOSABSABS AND BCONST COLON COMMA COMPONENT_ID COS DCONST DIVIDE DOT ELSE EQ EQUALS GE GT ICONST ID IF LBRACE LE LPAREN LT MINUS MOD MOTION MSGTYPE NE NOT OR PLUS PRINT RBRACE RECEIVE RPAREN SCONST SEMI SEND SIN SKIP SQRT TAN TIMES WHILE statement : statement SEMI statement\n                      | receive\n                      | send\n                      | if\n                      | while\n                      | assign\n                      | motion\n                      | print\n                      | skip receive   : RECEIVE LPAREN motion RPAREN  LBRACE actions RBRACEsend : SEND LPAREN COMPONENT_ID COMMA MSGTYPE COMMA expression RPARENif : IF LPAREN expression RPAREN LBRACE statement RBRACE ELSE LBRACE statement RBRACEwhile : WHILE LPAREN expression RPAREN LBRACE statement RBRACE assign : ID EQUALS LBRACE keyvalue RBRACE\n                   | ID DOT ID EQUALS expression\n                   | ID EQUALS expression  motion : MOTION LPAREN args RPAREN\n                   | MOTION print : PRINT LPAREN args RPARENskip : SKIP actions : actions COMMA actions\n                    | LPAREN MSGTYPE COMMA ID COMMA LBRACE statement RBRACE RPAREN  keyvalue : keyvalue COMMA keyvalue\n                     | SCONST COLON expression args : args COMMA args\n                | expressionexpression : expression PLUS expression\n                  | expression MINUS expression\n                  | expression TIMES expression\n                  | expression DIVIDE expression\n                  | expression MOD expression\n                  | expression AND expression\n                  | expression OR expression\n                  | expression GT expression\n                  | expression GE expression\n                  | expression LT expression\n                  | expression LE expression\n                  | expression EQ expression\n                  | expression NE expressionexpression : MINUS expression %prec UMINUS\n                      | SIN LPAREN expression RPAREN\n                      | COS LPAREN expression RPAREN\n                      | TAN LPAREN expression RPAREN\n                      | ABS LPAREN expression RPAREN\n                      | SQRT LPAREN expression RPAREN\n                      | NOT expressionexpression : LPAREN expression RPARENexpression : ICONST\n                      | DCONST\n                      | SCONST\n                      | BCONSTexpression : ID\n                      | ID DOT ID'
    
_lr_action_items = {'GE':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,61,61,61,61,-46,-40,61,61,61,61,61,-53,61,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,61,-44,-42,-41,-45,-43,61,61,]),'COS':([19,21,23,24,25,38,39,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,110,120,],[31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,]),'DIVIDE':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,62,62,62,62,62,-40,62,62,62,62,62,-53,62,62,-30,-31,62,62,62,62,62,62,62,62,62,-29,-47,62,-44,-42,-41,-45,-43,62,62,]),'DOT':([16,37,],[26,57,]),'SEMI':([1,3,4,5,7,11,12,13,14,15,17,27,29,33,34,36,37,49,52,58,59,79,88,90,91,92,93,94,95,96,97,98,99,100,101,102,103,108,111,112,113,114,115,116,119,121,125,127,132,137,139,140,],[18,-6,-8,-3,-5,-4,-20,-18,-2,-9,-7,18,-48,-50,-49,-51,-52,-16,-19,-46,-40,-17,-53,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,-14,-15,-44,-42,-41,-45,-43,18,18,-10,-13,-11,18,-12,18,]),'LE':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,65,65,65,65,-46,-40,65,65,65,65,65,-53,65,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,65,-44,-42,-41,-45,-43,65,65,]),'RECEIVE':([0,18,105,107,135,138,],[6,6,6,6,6,6,]),'GT':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,72,72,72,72,-46,-40,72,72,72,72,72,-53,72,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,72,-44,-42,-41,-45,-43,72,72,]),'BCONST':([19,21,23,24,25,38,39,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,110,120,],[36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,]),'IF':([0,18,105,107,135,138,],[10,10,10,10,10,10,]),'COMMA':([29,30,33,34,36,37,41,45,47,58,59,80,84,88,90,91,92,93,94,95,96,97,98,99,100,101,102,103,106,112,113,114,115,116,117,122,123,126,130,134,142,],[-48,53,-50,-49,-51,-52,-26,77,53,-46,-40,109,53,-53,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,120,-44,-42,-41,-45,-43,124,109,-24,131,124,136,-22,]),'DCONST':([19,21,23,24,25,38,39,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,110,120,],[34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,]),'EQUALS':([16,50,],[25,82,]),'$end':([1,3,4,5,7,11,12,13,14,15,17,27,29,33,34,36,37,49,52,58,59,79,88,90,91,92,93,94,95,96,97,98,99,100,101,102,103,108,111,112,113,114,115,116,125,127,132,139,],[0,-6,-8,-3,-5,-4,-20,-18,-2,-9,-7,-1,-48,-50,-49,-51,-52,-16,-19,-46,-40,-17,-53,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,-14,-15,-44,-42,-41,-45,-43,-10,-13,-11,-12,]),'COLON':([81,],[110,]),'WHILE':([0,18,105,107,135,138,],[8,8,8,8,8,8,]),'MINUS':([19,21,23,24,25,29,33,34,36,37,38,39,41,42,44,46,49,51,53,54,55,56,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,82,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,110,111,112,113,114,115,116,120,123,128,],[39,39,39,39,39,-48,-50,-49,-51,-52,39,39,66,39,66,66,66,39,39,39,39,39,66,-40,39,39,39,39,39,39,39,39,39,39,39,39,39,39,66,39,66,66,66,66,-53,66,66,-30,-31,66,66,-28,-27,66,66,66,66,66,-29,-47,39,66,-44,-42,-41,-45,-43,39,66,66,]),'ABS':([19,21,23,24,25,38,39,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,110,120,],[28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,]),'RPAREN':([13,29,30,33,34,36,37,41,43,44,46,47,58,59,74,79,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,112,113,114,115,116,128,141,],[-18,-48,52,-50,-49,-51,-52,-26,75,76,78,79,-46,-40,103,-17,112,-25,113,114,115,-53,116,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,-44,-42,-41,-45,-43,132,142,]),'MSGTYPE':([77,118,],[106,126,]),'TIMES':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,73,73,73,73,73,-40,73,73,73,73,73,-53,73,73,-30,-31,73,73,73,73,73,73,73,73,73,-29,-47,73,-44,-42,-41,-45,-43,73,73,]),'TAN':([19,21,23,24,25,38,39,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,110,120,],[40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,]),'EQ':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,64,64,64,64,-46,-40,64,64,64,64,64,-53,64,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,64,-44,-42,-41,-45,-43,64,64,]),'PRINT':([0,18,105,107,135,138,],[2,2,2,2,2,2,]),'SCONST':([19,21,23,24,25,38,39,42,48,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,109,110,120,],[33,33,33,33,33,33,33,33,81,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,81,33,33,]),'COMPONENT_ID':([22,],[45,]),'RBRACE':([3,4,5,7,11,12,13,14,15,17,27,29,33,34,36,37,49,52,58,59,79,80,88,90,91,92,93,94,95,96,97,98,99,100,101,102,103,108,111,112,113,114,115,116,117,119,121,122,123,125,127,130,132,137,139,140,142,],[-6,-8,-3,-5,-4,-20,-18,-2,-9,-7,-1,-48,-50,-49,-51,-52,-16,-19,-46,-40,-17,108,-53,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,-14,-15,-44,-42,-41,-45,-43,125,127,129,-23,-24,-10,-13,-21,-11,139,-12,141,-22,]),'ICONST':([19,21,23,24,25,38,39,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,110,120,],[29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,]),'AND':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,69,69,69,69,-46,-40,69,69,69,69,69,-53,69,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,69,-44,-42,-41,-45,-43,69,69,]),'LBRACE':([25,75,76,78,133,136,],[48,104,105,107,135,138,]),'SIN':([19,21,23,24,25,38,39,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,110,120,],[32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,]),'SEND':([0,18,105,107,135,138,],[9,9,9,9,9,9,]),'SKIP':([0,18,105,107,135,138,],[12,12,12,12,12,12,]),'MOTION':([0,18,20,105,107,135,138,],[13,13,13,13,13,13,13,]),'MOD':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,63,63,63,63,63,-40,63,63,63,63,63,-53,63,63,-30,-31,63,63,63,63,63,63,63,63,63,-29,-47,63,-44,-42,-41,-45,-43,63,63,]),'ELSE':([129,],[133,]),'NOT':([19,21,23,24,25,38,39,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,110,120,],[38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,]),'ID':([0,18,19,21,23,24,25,26,38,39,42,51,53,54,55,56,57,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,105,107,110,120,131,135,138,],[16,16,37,37,37,37,37,50,37,37,37,37,37,37,37,37,88,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,16,16,37,37,134,16,16,]),'PLUS':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,67,67,67,67,67,-40,67,67,67,67,67,-53,67,67,-30,-31,67,67,-28,-27,67,67,67,67,67,-29,-47,67,-44,-42,-41,-45,-43,67,67,]),'OR':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,68,68,68,68,-46,-40,68,68,68,68,68,-53,68,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,68,-44,-42,-41,-45,-43,68,68,]),'LT':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,70,70,70,70,-46,-40,70,70,70,70,70,-53,70,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,70,-44,-42,-41,-45,-43,70,70,]),'NE':([29,33,34,36,37,41,44,46,49,58,59,74,83,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,111,112,113,114,115,116,123,128,],[-48,-50,-49,-51,-52,71,71,71,71,-46,-40,71,71,71,71,71,-53,71,-35,-30,-31,-38,-37,-28,-27,-33,-32,-36,-39,-34,-29,-47,71,-44,-42,-41,-45,-43,71,71,]),'SQRT':([19,21,23,24,25,38,39,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,110,120,],[35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,]),'LPAREN':([2,6,8,9,10,13,19,21,23,24,25,28,31,32,35,38,39,40,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,104,110,120,124,],[19,20,21,22,23,24,42,42,42,42,42,51,54,55,56,42,42,60,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,118,42,42,118,]),}

_lr_action = {}
for _k, _v in _lr_action_items.items():
   for _x,_y in zip(_v[0],_v[1]):
      if not _x in _lr_action:  _lr_action[_x] = {}
      _lr_action[_x][_k] = _y
del _lr_action_items

_lr_goto_items = {'if':([0,18,105,107,135,138,],[11,11,11,11,11,11,]),'statement':([0,18,105,107,135,138,],[1,27,119,121,137,140,]),'while':([0,18,105,107,135,138,],[7,7,7,7,7,7,]),'receive':([0,18,105,107,135,138,],[14,14,14,14,14,14,]),'actions':([104,124,],[117,130,]),'assign':([0,18,105,107,135,138,],[3,3,3,3,3,3,]),'skip':([0,18,105,107,135,138,],[15,15,15,15,15,15,]),'send':([0,18,105,107,135,138,],[5,5,5,5,5,5,]),'keyvalue':([48,109,],[80,122,]),'motion':([0,18,20,105,107,135,138,],[17,17,43,17,17,17,17,]),'args':([19,24,53,],[30,47,84,]),'print':([0,18,105,107,135,138,],[4,4,4,4,4,4,]),'expression':([19,21,23,24,25,38,39,42,51,53,54,55,56,60,61,62,63,64,65,66,67,68,69,70,71,72,73,82,110,120,],[41,44,46,41,49,58,59,74,83,41,85,86,87,89,90,91,92,93,94,95,96,97,98,99,100,101,102,111,123,128,]),}

_lr_goto = {}
for _k, _v in _lr_goto_items.items():
   for _x, _y in zip(_v[0], _v[1]):
       if not _x in _lr_goto: _lr_goto[_x] = {}
       _lr_goto[_x][_k] = _y
del _lr_goto_items
_lr_productions = [
  ("S' -> statement","S'",1,None,None,None),
  ('statement -> statement SEMI statement','statement',3,'p_statement_program','parser.py',36),
  ('statement -> receive','statement',1,'p_statement_program','parser.py',37),
  ('statement -> send','statement',1,'p_statement_program','parser.py',38),
  ('statement -> if','statement',1,'p_statement_program','parser.py',39),
  ('statement -> while','statement',1,'p_statement_program','parser.py',40),
  ('statement -> assign','statement',1,'p_statement_program','parser.py',41),
  ('statement -> motion','statement',1,'p_statement_program','parser.py',42),
  ('statement -> print','statement',1,'p_statement_program','parser.py',43),
  ('statement -> skip','statement',1,'p_statement_program','parser.py',44),
  ('receive -> RECEIVE LPAREN motion RPAREN LBRACE actions RBRACE','receive',7,'p_receive_msg','parser.py',51),
  ('send -> SEND LPAREN COMPONENT_ID COMMA MSGTYPE COMMA expression RPAREN','send',8,'p_send_msg','parser.py',55),
  ('if -> IF LPAREN expression RPAREN LBRACE statement RBRACE ELSE LBRACE statement RBRACE','if',11,'p_if_code','parser.py',59),
  ('while -> WHILE LPAREN expression RPAREN LBRACE statement RBRACE','while',7,'p_while_code','parser.py',63),
  ('assign -> ID EQUALS LBRACE keyvalue RBRACE','assign',5,'p_assign_code','parser.py',67),
  ('assign -> ID DOT ID EQUALS expression','assign',5,'p_assign_code','parser.py',68),
  ('assign -> ID EQUALS expression','assign',3,'p_assign_code','parser.py',69),
  ('motion -> MOTION LPAREN args RPAREN','motion',4,'p_motion_exec','parser.py',78),
  ('motion -> MOTION','motion',1,'p_motion_exec','parser.py',79),
  ('print -> PRINT LPAREN args RPAREN','print',4,'p_print_function','parser.py',86),
  ('skip -> SKIP','skip',1,'p_skip_function','parser.py',90),
  ('actions -> actions COMMA actions','actions',3,'p_actions_tuple','parser.py',96),
  ('actions -> LPAREN MSGTYPE COMMA ID COMMA LBRACE statement RBRACE RPAREN','actions',9,'p_actions_tuple','parser.py',97),
  ('keyvalue -> keyvalue COMMA keyvalue','keyvalue',3,'p_keyvalue_dictionary','parser.py',104),
  ('keyvalue -> SCONST COLON expression','keyvalue',3,'p_keyvalue_dictionary','parser.py',105),
  ('args -> args COMMA args','args',3,'p_args_tuple','parser.py',114),
  ('args -> expression','args',1,'p_args_tuple','parser.py',115),
  ('expression -> expression PLUS expression','expression',3,'p_expression_binop','parser.py',124),
  ('expression -> expression MINUS expression','expression',3,'p_expression_binop','parser.py',125),
  ('expression -> expression TIMES expression','expression',3,'p_expression_binop','parser.py',126),
  ('expression -> expression DIVIDE expression','expression',3,'p_expression_binop','parser.py',127),
  ('expression -> expression MOD expression','expression',3,'p_expression_binop','parser.py',128),
  ('expression -> expression AND expression','expression',3,'p_expression_binop','parser.py',129),
  ('expression -> expression OR expression','expression',3,'p_expression_binop','parser.py',130),
  ('expression -> expression GT expression','expression',3,'p_expression_binop','parser.py',131),
  ('expression -> expression GE expression','expression',3,'p_expression_binop','parser.py',132),
  ('expression -> expression LT expression','expression',3,'p_expression_binop','parser.py',133),
  ('expression -> expression LE expression','expression',3,'p_expression_binop','parser.py',134),
  ('expression -> expression EQ expression','expression',3,'p_expression_binop','parser.py',135),
  ('expression -> expression NE expression','expression',3,'p_expression_binop','parser.py',136),
  ('expression -> MINUS expression','expression',2,'p_expression_unop','parser.py',165),
  ('expression -> SIN LPAREN expression RPAREN','expression',4,'p_expression_unop','parser.py',166),
  ('expression -> COS LPAREN expression RPAREN','expression',4,'p_expression_unop','parser.py',167),
  ('expression -> TAN LPAREN expression RPAREN','expression',4,'p_expression_unop','parser.py',168),
  ('expression -> ABS LPAREN expression RPAREN','expression',4,'p_expression_unop','parser.py',169),
  ('expression -> SQRT LPAREN expression RPAREN','expression',4,'p_expression_unop','parser.py',170),
  ('expression -> NOT expression','expression',2,'p_expression_unop','parser.py',171),
  ('expression -> LPAREN expression RPAREN','expression',3,'p_expression_group','parser.py',188),
  ('expression -> ICONST','expression',1,'p_expression_constant','parser.py',192),
  ('expression -> DCONST','expression',1,'p_expression_constant','parser.py',193),
  ('expression -> SCONST','expression',1,'p_expression_constant','parser.py',194),
  ('expression -> BCONST','expression',1,'p_expression_constant','parser.py',195),
  ('expression -> ID','expression',1,'p_expression_id','parser.py',199),
  ('expression -> ID DOT ID','expression',3,'p_expression_id','parser.py',200),
]
