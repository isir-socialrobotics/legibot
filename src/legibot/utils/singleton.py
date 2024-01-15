class Singleton(type):
    _instances = {}

    @staticmethod
    def get_instance_name(instance):
        return str(instance)[8:-2].split('.')[-1]

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            existing_instance = [cls.get_instance_name(instance) for instance in cls._instances]
            if cls.get_instance_name(cls) in existing_instance:
                raise Exception(f"Singleton class {cls.get_instance_name(cls)} already exists "
                                f"| fix path to singleton class")
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]