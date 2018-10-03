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
        self.n = 0
        self.msgs = set()
        self.mps = set()
        self.labels = dict()

    def write(self, text):
        self.file.write(text)
        self.file.write("\n")
        if self.debug:
            print(text)

    def write_indent(self, indent, string):
        tmp = ("".join("  " for i in range(0,indent))) + string
        self.write(tmp)

    def as_msg(self, msg):
        return "msg_" + str(msg)

    def as_mp(self, mp):
        return "mp_" + str(mp)

    def as_label(self, l):
        return "l_" + str(l)

    def condition_as_string(self, cond):
        # TODO better
        if cond == sp.false or cond == False:
            return "false"
        else:
            return "true"

    def collect_msgs(self, statement):
        if isinstance(statement, ast_inter.Statement):
            return { msg for c in statement.children for msg in self.collect_msgs(c) }
        elif isinstance(statement, ast_inter.Receive): # Action
            msgs = set()
            for a in statement.actions:
                msgs.add(a.str_msg_type)
                msgs.update(self.collect_msgs(a.program))
            return msgs
        elif isinstance(statement, ast_inter.If): # IfComponent
            return { msg for c in statement.if_list for msg in self.collect_msgs(c.program) }
        elif isinstance(statement, ast_inter.While):
            return self.collect_msgs(statement.program)
        elif isinstance(statement, ast_inter.Send):
            return set(statement.msg_type)
        elif isinstance(statement, ast_inter.Motion):
            return set()
        elif isinstance(statement, ast_inter.Print):
            return set()
        elif isinstance(statement, ast_inter.Skip):
            return set()
        elif isinstance(statement, ast_inter.Exit):
            return set()
        else:
            raise Exception("!??! " + str(statement))

    def collect_mps(self, statement):
        if isinstance(statement, ast_inter.Statement):
            return { mp for c in statement.children for mp in self.collect_mps(c) }
        elif isinstance(statement, ast_inter.Receive): # Action
            mps = self.collect_mps(statement.motion)
            for a in statement.actions:
                mps.update(self.collect_mps(a.program))
            return mps
        elif isinstance(statement, ast_inter.If): # IfComponent
            return { mps for c in statement.if_list for mp in self.collect_mps(c.program) }
        elif isinstance(statement, ast_inter.While):
            return self.collect_mps(statement.program)
        elif isinstance(statement, ast_inter.Send):
            return set()
        elif isinstance(statement, ast_inter.Motion):
            return { statement.value }
        elif isinstance(statement, ast_inter.Print):
            return set()
        elif isinstance(statement, ast_inter.Skip):
            return set()
        elif isinstance(statement, ast_inter.Exit):
            return set()
        else:
            raise Exception("!??! " + str(statement))

    def print_mtype_decl(self):
        str_msgs = { self.as_msg(m) for m in self.msgs }
        str_mps = { self.as_mp(m) for m in self.mps }
        str_lbs = { self.as_label(l) for l in self.labels }
        strs = str_msgs.union(str_mps).union(str_lbs)
        self.write("mtype = { " + ", ".join(strs) + " }")

    def print_scheduling_decl(self):
        for i in range(0, self.n):
            self.write("int busy_for_" + str(i) + " = 0")
            self.write("mtype doing_" + str(i))
            self.write("mtype loc_" + str(i))
            self.write("bool terminated_" + str(i) + " = false")
            self.write("#define set_mp_" + str(i) + "(label, mp, time) { loc_"+str(i)+" = label; doing_"+str(i)+" = mp; busy_for_"+str(i)+" = time; busy_for_"+str(i)+" == 0 }")
            self.write("chan channel_" + str(i) + " = [0] of { mtype }")

    def print_scheduler(self):
        self.write("active proctype time_manager() {")
        self.write("    do")
        self.write("    :: " + " && ".join("busy_for_"+str(i)+" > 0" for i in range(0,self.n)) + " ->")
        self.write("        d_step {")
        self.write("            printf(\"MP\");")
        for i in range(0, self.n):
            self.write("            if")
            for m  in self.mps:
                self.write("            :: doing_"+str(i)+" == " + self.as_mp(m) + " ->")
                self.write("                if")
                for l in self.labels:
                    self.write("                :: loc_"+str(i)+" == " + self.as_label(l) + " ->")
                    self.write("                    printf(\", "+str(i)+" at " + str(l) + " doing " + str(m) + " for %d\", busy_for_"+str(i)+")")
                self.write("                fi")
            self.write("            fi;")
        self.write("            if")
        for i in range(0, self.n):
            self.write("            :: " + " && ".join("busy_for_"+str(i)+" <= busy_for_"+str(j) for j in range(0,self.n) if j != i) + " ->")
            for j in range(0, self.n):
                if i != j:
                    self.write("                busy_for_"+str(j)+" = busy_for_"+str(j)+" - busy_for_"+str(i)+";")
            self.write("                busy_for_"+str(i)+" = 0")
        self.write("            fi;")
        self.write("            printf(\"\\n\")")
        self.write("        }")
        self.write("    :: " + " && ".join("terminated_"+str(i) for i in range(0,self.n)) + " ->")
        self.write("        break")
        self.write("    od")
        self.write("}")

    def print_stmt(self, i, name_to_id, statement, indent = 1, end_of_line = ""):
        if isinstance(statement, ast_inter.Statement):
            for c in statement.children:
                self.print_stmt(i, name_to_id, c, indent + 4, ";")
        elif isinstance(statement, ast_inter.Receive): # Action
            self.write_indent(indent, "do")
            for a in statement.actions:
                self.write_indent(indent, ":: channel_" + str(i) + "?" + self.as_msg(a.str_msg_type) + " ->")
                self.print_stmt(i, name_to_id, a.program, indent + 1, end_of_line = ";")
                self.write_indent(indent + 1, "break")
            if statement.motion != None:
                self.write_indent(indent, ":: timeout ->")
                self.print_stmt(i, name_to_id, statement.motion, indent + 1)
            self.write_indent(indent, "od" + end_of_line)
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
            self.write_indent(indent, "channel_" + str(name_to_id[statement.comp]) + "!" + self.as_msg(statement.msg_type) + end_of_line)
        elif isinstance(statement, ast_inter.Motion):
            self.write_indent(indent, "set_mp_" + str(i) + "(" + self.as_label(statement.get_label()) + ", " + self.as_mp(statement.value) + ", 1)"+ end_of_line)
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
        self.write("    LEXIT_" + str(i) + ": terminated_"+str(i)+" = true")
        self.write("}")

    def parse_mp(self, text):
        #sample: 0 at loc doing m_Fold for 1
        raw_mps = text.split(',')
        mps = {}
        for i in raw_mps[1:]:
            parts = i.strip().split(' ')
            mps[int(parts[0])] = (parts[2], parts[4], int(parts[6]))
        return mps

    def call_spin(self):
        assert(self.dir != None)
        assert(self.file != None)
        old_path = os.getcwd()
        try:
            os.chdir(self.dir)
            # call spin and parse the result
            command1 = ["spin", "-a", "model.pml"]
            call_spin = subprocess.run(command1)
            assert(call_spin.returncode == 0)
            command2 = ["gcc", "-o", "pan", "-DSAFETY", "-DPRINTF", "pan.c"]
            call_gcc = subprocess.run(command2)
            assert(call_gcc.returncode == 0)
            command3 = ["./pan", "-m10000", "-c1", "-w19"]
            call_pan = subprocess.run(command3, stdout=subprocess.PIPE, encoding="ascii")
            if call_pan.returncode == 0:
                print("# spin result is:")
                lines = call_pan.stdout.split('\n')
                mps = []
                err = []
                for l in lines:
                    print(l)
                    if l.startswith("MP"):
                        mps.append(self.parse_mp(l))
                    elif l.startswith("pan:"):
                        err.append(l)
                if len(err) > 1:
                    print("ERROR detected by spin")
                    command4 = ["spin", "-t", "model.pml"]
                    call_spin = subprocess.run(command4)
                    return (mps, False)
                else:
                    print("spin check succeeded")
                    return (mps, True)
            else:
                raise Exception("error running spin")
        finally:
            os.chdir(old_path)

    def check_motion(self, mps):
        for mp in mps:
            print("mp:", mp)
        raise Exception("TODO check_motion")

    def check(self):
        self.n = len(self.processes)
        self.msgs = { m for name, prog in self.processes for m in self.collect_msgs(prog) }
        self.mps = { m for name, prog in self.processes for m in self.collect_mps(prog) }
        with tempfile.TemporaryDirectory() as tmpdirname:
            self.dir = tmpdirname
            self.file = open(tmpdirname + "/model.pml", 'w')
            try:
                i = 0
                name_to_id = {}
                for name, program in self.processes:
                    name_to_id[name] = i
                    labels = program.label_as_root()
                    self.labels.update(labels)
                    i = i + 1
                self.print_mtype_decl()
                self.print_scheduling_decl()
                i = 0
                for name, program in self.processes:
                    self.print_process(i, name_to_id, name, program)
                    i = i + 1
                self.print_scheduler()
            finally:
                self.file.close()
            mps, result = self.call_spin()
            self.file = None
            self.dir = None
            return result and self.check_motion(mps)
