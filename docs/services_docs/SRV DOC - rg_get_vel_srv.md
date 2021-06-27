\page DOCSRV_rg_get_vel_srv rg_get_vel_srv

# SERVICE - rg_get_vel_srv

**Provided by:**

- `rg_services.cpp`
- `rg_services_py.py`

**Used by:**

- `rg_controller.cpp`
- `rg_controller_py.py`

## Semantic

This service message is used for obtaining the linear velocity towards the point. Only linear plananr twists without angular velocity are considered. 

See `rg_services_py.py` (function `rg_get_vel_callback`) or `rg_services.cpp` (function `rg_get_velocity_callback`) for more details about the performed computation (very simple). 

## Service code

```yaml
# service file name 'rg_check_target_srv'

# actual position
geometry_msgs/Point x

# target position
geometry_msgs/Point xt

---

# the twist to send to 'cmd_vel'
geometry_msgs/Twist vel
```
