import os
import yaml


def load_config():    
    """
    Load configuration from a YAML file.

    Parameters:
        path (str): Full path to the YAML configuration file.

    Returns:
        config (dict): Dictionary containing all configuration parameters.
    """
    project_dir = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(project_dir, '..', 'config', 'config.yaml')
    path = os.path.abspath(path)
    
    if not os.path.exists(path):
        raise FileNotFoundError(f"Config file not found at: {path}")

    with open(path, "r") as f:
        config = yaml.safe_load(f)
    return config