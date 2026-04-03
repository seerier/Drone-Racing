"""Fix for wandb/pydantic compatibility with Isaac Sim.

This module patches inspect.getfile to handle namespace packages properly,
which fixes the pydantic issue when importing wandb in Isaac Sim environment.
"""

import inspect
import os


def apply_wandb_patch():
    """Apply the wandb compatibility patch for Isaac Sim.

    This patches inspect.getfile to handle namespace packages that pydantic
    encounters when importing wandb in Isaac Sim environment.
    """
    # Store the original getfile function
    _original_getfile = inspect.getfile

    def _patched_getfile(object):
        """Patched version of inspect.getfile that handles namespace packages."""
        try:
            return _original_getfile(object)
        except TypeError as e:
            # Handle the case where object is detected as a "built-in module"
            # This happens with namespace packages in Isaac Sim
            if "built-in module" in str(e):
                # For namespace packages, return a dummy path
                # This satisfies pydantic's inspection requirements
                return "<namespace-package>"
            # Re-raise if it's a different TypeError
            raise

    # Apply the patch
    inspect.getfile = _patched_getfile
    print("[INFO] Applied wandb compatibility patch for Isaac Sim environment")


# Apply the patch when this module is imported
apply_wandb_patch()