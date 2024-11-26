import scriptcontext as sc  # type: ignore
from compas.scene import SceneObject
from compas.scene import Scene

from compas_ghpython import create_id


def cache_scene_object(model, kwargs, components, sceneobject_type=None, debug=False):
    """Creates or retrieves a SceneObject from the sticky cache.

    This is intended for caching RobotCellObjects and their constituent SceneObjects.

    If the SceneObject is not in the cache, it will be created and stored in the cache.
    The SceneObject is created using ``compas.scene.Scene.add(model, **kwargs)``.

    The cache scope is limited to the current component instance, such as a Grasshopper
    component. Meaning that it is possible to have multiple cached SceneObjects in the
    same GH document, allowing multiple visualizations of the same model.

    Parameters
    ----------
    model : object
        The model for which the SceneObject is created.
    kwargs : dict
        The keyword arguments to be passed to the SceneObject creation.
    components : `ghpythonlib.componentbase.executingcomponent`
        The components instance. Use `self` in advanced (SDK) mode and `ghenv.Components` otherwise.
    debug : bool, optional
        If True, debug messages will be printed. Default is False.

    Returns
    -------
    compas.scene.SceneObject
        The SceneObject created or retrieved from the cache.

    """

    def debug_print(s):
        if debug:
            print(s)

    # Create hash (scope limited to this component) for cacheing SceneObject
    sticky_id = create_id(components, "")

    # We use a tuple to store the SceneObject (first value is the hash of the model)
    if not sc.sticky.has_key(sticky_id):
        sc.sticky[sticky_id] = (None, None)
    cached_hash, cached_scene_object = sc.sticky[sticky_id]

    # The ID function is much faster than the sha256()
    model_hash = str(id(model))
    kwargs_hash = "_".join([str(k) + str(v) for k, v in kwargs.items()])
    hash = model_hash + "_" + kwargs_hash
    debug_print("Model and Kwargs Hash: '{}'".format(hash))

    # Retrieve Cached SceneObject from sticky
    scene_object = None
    if cached_hash == hash and cached_scene_object is not None:
        scene_object = cached_scene_object
        # Double check if the item is the same
        if scene_object.item is model:
            debug_print(
                "Retrieved SceneObject '{}' from cache for '{}'".format(
                    type(scene_object).__name__, type(model).__name__
                )
            )
    # Create New SceneObject
    else:
        if sceneobject_type:
            scene_object = SceneObject(
                item=model,
                sceneobject_type=sceneobject_type,
                **kwargs,
            )
        else:
            scene = Scene()
            scene_object = scene.add(model, **kwargs)
        debug_print("New SceneObject {} created for {}".format(type(scene_object).__name__, type(model).__name__))
        sc.sticky[sticky_id] = (hash, scene_object)

    return scene_object
