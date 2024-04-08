import os
import shutil
import subprocess
import sys
import time
from crow_utils.crow_config import get_config_file, get_lib
from crow_utils.net import wait_for_open

class FusekiBackend:

    def __init__(self, config, dbconf, log):
        self.config = config
        self.log = log
        self.dbconf = dbconf

    def start(self):
        if "fuseki_path" not in self.config:
            self.log("Fuseki server path NOT found in the config, assuming server was started manually.")
            return

        self.log("Fuseki server path found in the config, attempting to start the server.")
        self.fuseki_path = get_lib(self.config['fuseki_path'])
        self.fuseki_run_cmd = os.path.join(self.fuseki_path, 'fuseki')
        if os.path.exists(self.fuseki_run_cmd):
            self.log(f'Running fuseki as service from {self.fuseki_run_cmd}...')
        else:
            raise Exception(f'Fuseki executable not found in: {self.fuseki_run_cmd}!')

        self.start_fuseki()

        wait_for_open(ip="127.0.0.1", port=int(self.dbconf.port))


    def start_fuseki(self):
        fuseki_env = {
                **os.environ,
                # "JAVA": '/home/imitlearn/miniconda3/envs/ros2_humble/bin/java',
                "FUSEKI_ARGS": f"--port {self.dbconf.port} --update --tdb2 --loc run /{self.dbconf.database}",
            }

        if self.get_fuseki_status():  # if fuseki is running, restart it
            self.log('Fuseki is already running! Trying to stop it.')
            self.stop()

        self.compactify_fuseki()

        ret = subprocess.run(' '.join([self.fuseki_run_cmd, 'start']), stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, check=True, env=fuseki_env)
        if ret.returncode > 0:
            raise RuntimeError(f"Fuseki returned an error code: {ret.returncode}.\nThe output of the run command: {ret.stdout.decode('utf-8')}")
        else:
            if "fail" in str(ret.stdout):
                self.log(f"Fuseki service somehow failed but the returncode was 0 (indicating no error, do you have Java?). Read the output of the run command and decide what to do:\n{ret.stdout.decode('utf-8')}")
            else:
                self.log(f"{ret.stdout.decode('utf')}\nFuseki service started on port {self.dbconf.port}.")

    def stop(self):
        if self.fuseki_run_cmd and self.get_fuseki_status():
            ret = subprocess.run(' '.join([self.fuseki_run_cmd, 'stop']), stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, check=True)
            if ret.returncode > 0:
                raise RuntimeError(f"Trying to stop Fuseki returned an error code: {ret.returncode}.\nThe output of the run command: {ret.stdout}")
            else:
                self.log(f"{ret.stdout.decode('utf-8')}\nFuseki service stopped.")

    def clear(self, onto):
        onto.update("""
                        DELETE {
                            ?s ?p ?o
                        }
                        WHERE {
                            ?s ?p ?o .
                        }
                        """)

    def compactify_fuseki(self):
        # fuseki_tool_path = '~/packages/apache-jena-4.2.0/'
        self.log("Trying to compactify fuseki database (this will fail if fuseki is already running)...")
        fuseki_tool_path = self.fuseki_path.replace("fuseki-", "") # try to extract fuseki tools from the fuseki_path
        fuseki_tool_path = os.path.expanduser(fuseki_tool_path)
        compactor_path = os.path.join(fuseki_tool_path, 'bin/tdb2.tdbcompact')
        if not os.path.exists(compactor_path):
            self.log(f"Could not find Fuseki compactor at '{compactor_path}'")
            return
        try:
            ret = subprocess.run(' '.join([compactor_path, '--loc=' + os.path.join(self.fuseki_path + '/run/')]), stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, check=True)
            if ret.returncode > 0:
                raise RuntimeError(f"Tried to compactify the Fuseki database returned an error code: {ret.returncode}.\nThe output of the run command: {ret.stdout}")
            else:
                self.log(f"{ret.stdout.decode('utf-8')}\nFuseki database compactified.")
            #imitrob@aurora:~/packages/apache-jena-4.2.0/bin$ ./tdb2.tdbcompact --loc=../../apache-jena-fuseki-4.2.0/run/
        except BaseException as e:
            self.log(f"Tried to compactify the Fuseki database, but failed because: {e}")

    def get_fuseki_status(self) -> bool:
        ret = subprocess.run(' '.join([self.fuseki_run_cmd, 'status']), stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, check=True)
        if ret.returncode > 0:
            raise RuntimeError(f"Trying to get Fuseki status returned an error code: {ret.returncode}.\nThe output of the run command: {ret.stdout.decode('utf-')}")
        else:
            return "is running" in str(ret.stdout)

    @staticmethod
    def prepareQuery(query, *args, **kwargs):
        return query
