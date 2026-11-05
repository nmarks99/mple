from math import radians
import numpy as np
from epics import caget
import time
from mple import Mple

PREFIX = "mpleUR:"

def main():
    mple = Mple(f"{PREFIX}mple:")

    count = 0
    while True:
        time.sleep(0.2)
        testq = np.radians(caget(PREFIX + "Receive:Joints"))
        result = mple.validate(testq)
        if (result["valid"]):
            print(f"{count}: No collision")
        else:
            print(f'{count}: {result["description"]}')
        count += 1

if __name__ == "__main__":
    main()
