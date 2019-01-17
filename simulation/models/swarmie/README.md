To regenerate a rover's model.sdf from the erb template:
```
erb -T - rovername=name rovercolor=color -- model.sdf.erb > model.sdf
```

NOTE: You should use `misc/gen_rover_models.sh` to generate the all of the default
rover models.
