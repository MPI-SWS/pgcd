import ast_inter
import sympy as sp

import subprocess
import tempfile
import os

#TODO 

# model-checking messages
class McMessages:

    def __init__(self, processes, debug = False):
        self.debug = debug
        self.processes = processes
        self.dir = None
        self.file = None

    def write(self, text):
        self.file.write(test)
        self.file.write("\n")
        if self.debug:
            text2 = text.replace("%", "%%")
            print(text2)

    def write_indent(self, indent, string):
        tmp = ("".join("    " for i in range(0,indent))) + string
        self.write(tmp)

    def as_msg(self, msg):
        return "msg_" + str(msg)

    def as_mp(self, mp):
        return "mp_" + str(mp)

    def condition_as_string(self, cond):
        # TODO better
        return "true"

    def collect_msgs(self, program):
        ...

    def collect_mps(self, program):
        ...

    def print_mtype_decl(self, msgs, mps):
        str_msgs = { self.as_msg(m) for m in msgs }
        str_mps = { self.as_mp(m) for m in mps }
        assert len(str_msgs.intersection(str_mps)) == 0
        strs = str_msgs.union(str_mps)
        self.write("mtype = { " + ", ".join(strs) + " }")

    def print_scheduling_decl(self, n):
        for i in range(0, n):
            self.write("int busy_for_" + str(i) + " = 0")
            self.write("mtype doing_" + str(i))
            self.write("bool terminated_" + str(i) + " = false")
            self.write("#define set_mp_" + str(i) + "(mp, time) { doing_"+str(i)+" = mp; busy_for_"+str(i)+" = time; busy_for_"+str(i)+" == 0 }")

    def print_scheduler(self, n):
        self.write("active proctype time_manager() {")
        self.write("    do")
        self.write("    :: " + " && ".join("busy_for_"+str(i) for i in range(0,n)) + " ->")
        self.write("        d_step {")
        self.write("            printf(\"# start MP\");")
        for i in range(0, n):
            self.write("            if")
            for m  in mps:
                self.write("            :: doing_"+str(i)+" == " + self.as_mp(m) + " ->")
                self.write("                printf(\""+str(i)+" doing " + str(idle) + " for %d\\n\", busy_for_"+str(i)+")")
            self.write("            fi;")
        self.write("            if")
        for i in range(0, n):
            self.write("            :: " + " && ".join("busy_for_"+str(i)+" > busy_for_"+str(j) for j in range(0,n) if j != i) + " ->")
            for j in range(0, n):
                if i != j:
                    self.write("                busy_for_"+str(j)+" = busy_for_"+str(j)+" - busy_for_"+str(i)+";")
            self.write("                busy_for_"+str(i)+" = 0")
        self.write("            fi;")
        self.write("            printf(\"# end MP\")")
        self.write("        }")
        self.write("    :: " + " && ".join("terminated_"+str(i) for i in range(0,n)) + " ->")
        self.write("        break")
        self.write("    od")
        self.write("}")

    def print_stmt(self, i, name_to_id, statement, indent = 1, end_of_line = ""):
        if isinstance(statement, ast_inter.Statement):
            for c in statement.children:
                self.print_stmt(i, name_to_id, c, indent + 4, ";")
        elif isinstance(statement, ast_inter.Receive): # Action
            self.write_indent(indent, "if")
            for a in self.actions:
                self.write_indent(indent, ":: channel_" + str(i) + "?" + statement.str_msg_type + " ->")
                self.print_stmt(i, name_to_id, statement.program, indent + 1)
            if statement.motion != None:
                self.write_indent(indent, ":: timeout ->")
                self.print_stmt(i, name_to_id, statement.motion, indent + 1)
            self.write_indent(indent, "fi" + end_of_line)
        elif isinstance(statement, ast_inter.If): # IfComponent
            self.write_indent(indent, "if")
            for c in statement.if_list:
                self.write_indent(indent, ":: " + self.condition_as_string(c.condition) + " ->")
                self.print_stmt(i, name_to_id, c.program, indent + 1)
            self.write_indent(indent, "fi" + end_of_line)
        elif isinstance(statement, ast_inter.While):
            self.write_indent(indent, "do")
            self.write_indent(indent, "::" + self.condition_as_string(statement.condition) + " ->")
            self.print_stmt(i, name_to_id, statement.program, indent + 1)
            self.write_indent(indent, "::" + self.condition_as_string(sp.Not(statement.condition)) + " ->")
            self.write_indent(indent + 1, "break")
            self.write_indent(indent, "od" + end_of_line)
        elif isinstance(statement, ast_inter.Send):
            self.write_indent(indent, "channel_" + str(name_to_id(statement.comp)) + "!" + str(statement.msg_type) + end_of_line)
        elif isinstance(statement, ast_inter.Print):
            self.write_indent(indent, "skip" + end_of_line)
        elif isinstance(statement, ast_inter.Skip):
            self.write_indent(indent, "skip" + end_of_line)
        elif isinstance(statement, ast_inter.Exit):
            self.write_indent(indent, "goto LEXIT_" + str(i) + end_of_line)
        else:
            raise Exception("!??! " + str(statement))

    def print_process(self, i, name_to_id, name, proc):
        self.write("active proctype " + name + "() {")
        self.print_stmt(i, name_to_id, proc, 1)
        self.write("    LEXIT_" + str(i) + ": skip")
        self.write("}")

    def call_spin(self):
        assert(self.dir != None)
        assert(self.file != None)
        old_path = os.getcwd()
        try:
            os.chdir(self.dir)
            # TODO call spin and parse the result
            command1 = ["spin", "-a", "model.pml"]
            call_spin = subprocess.run(command1)
            assert(call_spin.returncode == 0)
            command2 = ["gcc", "-o", "pan", "-DSAFETY", "-DPRINTF", "pan.c"]
            call_gcc = subprocess.run(command2)
            assert(call_gcc.returncode == 0)
            command3 = ["./pan", "-m10000", "-c1", "-w19"]
            call_pan = subprocess.run(command3, stdout=subprocess.PIPE)
            if call_pan.returncode == 0:
                raise Exception("TODO analysis spin result")
            else:
                raise Exception("spin found an error")
        finally:
            os.chdir(old_path)

    def check(self):
        n = len(self.processes)
        msgs = { m for name, prog in self.processes for m in self.collect_msgs(prog) }
        mps = { m for name, prog in self.processes for m in self.collect_mps(prog) }
        with tempfile.TemporaryDirectory() as tmpdirname:
            self.dir = tmpdirname
            self.file = os.open("model.pml", os.O_CREAT | os.O_RDWR, dir_fd=tmpdirname)
            self.print_mtype_decl(msgs, mps)
            self.print_scheduling_decl(n)
            i = 0
            name_to_id = {}
            for name, program in self.processes:
                name_to_id[name] = i
                i = i + 1
                self.write("chan channel_" + str(i) + " = [0] of { mtype }")
            i = 0
            for name, program in self.processes:
                self.print_process(i, name_to_id, name, program)
                i = i + 1
            self.print_scheduler(n)
            self.file.close()
            result = self.call_spin()
            self.dir = None
            self.file = None
            return result
