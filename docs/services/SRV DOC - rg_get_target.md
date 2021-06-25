\page DOCSRV_rg_get_target_srv rg_get_target_srv

# SERVICE - rg_get_target_srv

**Provided by:**

- `rg_services.cpp`
- `rg_services_py.py`

**Used by:**

- `rg_controller.cpp`
- `rg_controller_py.py`

## Semantic

The robot controller calls this service when it needs a new target to reach, which must be *inside* the square space of the simulation environment. 

The controller sends the bounds to the service, and it replies with a `geometry_msgs/Point` message which is the randomly generated target inside the space. 

## Service code

```yaml
##	service file name 'rg_get_target_srv'
#		Request : the limits from the origin
#   	the space is an area (xmax - xmin)*(ymax - ymin) rectangle, centered in the origin

float64 xmin
float64 xmax
float64 ymin
float64 ymax

---

# Response: the new target to be reached
geometry_msgs/Point xt
```