# ::: compas_fab.rhino
    options:
      # The scene objects here are thin subclasses of the base objects in
      # `compas_fab.scene`; rendering their inherited methods would duplicate
      # those (e.g. `BaseRobotModelObject.draw`) across the ghpython/rhino/scene
      # pages and break cross-references. The methods are documented on the bases.
      inherited_members: false
