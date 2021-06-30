#!/usr/bin/python3

import paramiko

class Proxy():

    def __init__(self, hostname, username, password):
        self.client = paramiko.SSHClient()
        self.client.load_system_host_keys()
        self.client.set_missing_host_key_policy(paramiko.WarningPolicy())
        self.client.connect(hostname, 22, username, password)

    def __del__(self):
        self.client.close()

    def mk_command(self, command, args):
        cmd = command
        for a in args:
            cmd += ' '
            cmd += str(a)
        #print("command", cmd)
        return cmd

    def exec_bloquing(self, command, args, inpt = None):
        stdin, stdout, stderr = self.client.exec_command(self.mk_command(command, args))
        if inpt != None:
            stdin.sendall(inpt)
        status = stdout.channel.recv_exit_status()
        return (status, stdout, stderr)

    def exec_nonbloquing(self, command, args):
        stdin, stdout, stderr = self.client.exec_command(self.mk_command(command, args))
        return (stdin, stdout, stderr)
