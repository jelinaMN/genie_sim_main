
# Copyright (c) 2023-2025, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

import os, json
from pathlib import Path
from datetime import datetime

from base_utils.logger import Logger

logger = Logger()  # Create singleton instance


def check_and_fix_env():
    # check SIM_REPO_ROOT
    env_root_path = os.getenv("SIM_REPO_ROOT")
    if not env_root_path:
        current_dir = Path(__file__).resolve().parent.parent
        env_root_path = current_dir.parent
        os.environ["SIM_REPO_ROOT"] = env_root_path.as_posix()
        logger.warning(
            f"Warning: env [SIM_REPO_ROOT] empty, will use default: {env_root_path}"
        )
    else:
        logger.info(f"using env SIM_REPO_ROOT={env_root_path}")

    if not os.path.exists(env_root_path):
        os.makedirs(env_root_path, exist_ok=True)

    # check SIM_ASSETS
    assets_path = os.getenv("SIM_ASSETS")
    if not assets_path:
        assets_path = "~/assets"
        os.environ["SIM_ASSETS"] = assets_path
        logger.warning(
            f"Warning: env [SIM_ASSETS] empty, will use default: {assets_path}"
        )
    else:
        logger.info(f"using env SIM_ASSETS={assets_path}")

    if not os.path.exists(assets_path):
        os.makedirs(assets_path, exist_ok=True)

    target_path = os.path.join(env_root_path, "assets")
    if os.path.exists(target_path):
        os.remove(os.path.join(env_root_path, "assets"))
    os.symlink(assets_path, target_path, target_is_directory=True)


def benchmark_root_path():
    env_root_path = os.getenv("SIM_REPO_ROOT")
    return os.path.join(env_root_path, "benchmark")


def benchmark_bddl_path():
    env_root_path = os.getenv("SIM_REPO_ROOT")
    return os.path.join(env_root_path, "benchmark", "bddl")


def load_json(json_file):
    if not os.path.exists(json_file):
        raise ValueError("Json file not found: {}".format(json_file))
    with open(json_file) as f:
        return json.load(f)
    return None


def generate_new_file_path(dir_path, prefix_name, suffix="json"):
    count = 0
    while os.path.exists(os.path.join(dir_path, f"{prefix_name}_{count:02d}.{suffix}")):
        count += 1
    new_filename = f"{prefix_name}_{count:02d}.{suffix}"
    return os.path.join(dir_path, new_filename)


def TIMENOW():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
