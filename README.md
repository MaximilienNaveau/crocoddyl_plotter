Intended usage:

On crocoddyl side:
```python

server = crocoddyl_plotter.CrocoddylPlotterServer()

for iteration in range(100000):
	# ...
	ocp.solve(...)
	server.send(ocp.problem, iteration)
	# After this stage, the OCP datas are not meaningful anymore. The matrices have been swapped and will be sent in another thread.
```

In another terminal
```
crocoddyl_plotter_client_gui
```
