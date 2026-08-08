from mple import Mple
from p4p import Value, Type
import time

def main():
    mple = Mple("mpleTest:mple:")

    StatusMsg = Value(Type([
        ('running', 'd'),
        ('iter', 'd'),
        ('max', 'd'),
    ]))

    try:
        while True:
            result = mple.ctxt.get(f"{mple.prefix}Status", StatusMsg)
            print(f"Running = {result["running"]}")
            print(f"Iteration = {result["iter"]}")
            print(f"Max Iteration = {result["max"]}\n")
            #  if (result["max"] > 0):
                #  prog = (result["iter"] / result["max"]) * 100
                #  print(f"Progress: {prog:.1f}%")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Quitting")
        return

if __name__ == "__main__":
    main()
