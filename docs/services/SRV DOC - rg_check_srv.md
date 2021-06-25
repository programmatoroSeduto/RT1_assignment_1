\page DOCSRV_rg_check_srv rg_check_srv

# SERVICE - rg_check_srv

**Provided by:**

- `rg_services.cpp`
- `rg_services_py.py`

**Used by:**

- `rg_controller.cpp`
- `rg_controller_py.py`

## Semantic

The service simply checks if the actual position, i.e. *X*, is near enough (that is, under a given tolerance) to the target, i.e. *Xt*. 

## Service code

```yaml
# service file name 'rg_check_target_srv'

# actual position
geometry_msgs/Point x

# target position
geometry_msgs/Point xt

---

# is the goal "reached" (i.e. near under a certain tolerance)
bool reached
```
