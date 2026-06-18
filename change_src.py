Import("env")
import os, shutil

# Point PlatformIO at the arm/ directory as source root
src = os.path.join(env["PROJECT_DIR"], "arm")
env.Replace(SRC_DIR=src)
