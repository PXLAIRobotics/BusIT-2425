# Troubleshooting

---

> Package 'package_1' not found

**Solution:** You forgot to 

```
cd ~/Projects/ros2_workspace
colcon build
source install/setup.bash
```

---


---

`colcon build` output:

> Summary: 0 packages finished [0.97s]
  1 package failed: package_1
  1 package aborted: example_package
  1 package had stderr output: package_1
Command '['/usr/bin/python3', '-c', 'import sys;from contextlib import suppress;exec("with suppress(ImportError):    from setuptools.extern.packaging.specifiers    import SpecifierSet");exec("with suppress(ImportError):    from packaging.specifiers import SpecifierSet");from distutils.core import run_setup;dist = run_setup(    \'setup.py\', script_args=(\'--dry-run\',), stop_after=\'config\');skip_keys = (\'cmdclass\', \'distclass\', \'ext_modules\', \'metadata\');data = {    key: value for key, value in dist.__dict__.items()     if (        not key.startswith(\'_\') and         not callable(value) and         key not in skip_keys and         key not in dist.display_option_names    )};data[\'metadata\'] = {    k: v for k, v in dist.metadata.__dict__.items()     if k not in (\'license_files\', \'provides_extras\')};sys.stdout.buffer.write(repr(data).encode(\'utf-8\'))']' returned non-zero exit status 1.
	
**Solution:** You forgot a comma in your `setup.py` file

---
