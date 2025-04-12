# -*- coding: utf-8 -*-

class Parameter:
    """Parameter class for storing parameter information"""
    def __init__(self, name, value, param_type, description="", value_range=None):
        """
        Initialize parameter
        
        Args:
            name (str): Parameter name
            value: Parameter value
            param_type (type): Parameter type
            description (str): Parameter description
            value_range (tuple): Parameter value range in format (min, max)
        """
        self.name = name
        self._value = value
        self.type = param_type
        self.description = description
        self.value_range = value_range
        
    @property
    def value(self):
        """Get parameter value"""
        return self._value
    
    @value.setter
    def value(self, new_value):
        """Set parameter value with type and range checking"""
        # Type conversion
        try:
            typed_value = self.type(new_value)
        except (ValueError, TypeError):
            raise ValueError(f"Value {new_value} for parameter {self.name} cannot be converted to type {self.type.__name__}")
        
        # Range checking
        if self.value_range is not None:
            min_val, max_val = self.value_range
            if typed_value < min_val or typed_value > max_val:
                raise ValueError(f"Value {typed_value} for parameter {self.name} is out of range {self.value_range}")
        
        self._value = typed_value
        
    def to_dict(self):
        """Convert parameter to dictionary for JSON serialization"""
        return {
            "name": self.name,
            "value": self.value,
            "type": self.type.__name__,
            "description": self.description,
            "value_range": self.value_range
        }


class ParamManager:
    """Parameter manager for managing all parameters"""
    def __init__(self):
        """Initialize parameter manager"""
        self.params = {}
        
    def register(self, name, value, param_type=None, description="", value_range=None):
        """
        Register parameter
        
        Args:
            name (str): Parameter name
            value: Initial parameter value
            param_type (type, optional): Parameter type, auto-inferred if None
            description (str): Parameter description
            value_range (tuple): Parameter value range in format (min, max)
            
        Returns:
            Parameter: Registered parameter object
        """
        # Auto-infer type if not specified
        if param_type is None:
            param_type = type(value)
            
        # Create parameter object
        param = Parameter(name, value, param_type, description, value_range)
        self.params[name] = param
        return param
    
    def get(self, name):
        """
        Get parameter value
        
        Args:
            name (str): Parameter name
            
        Returns:
            Parameter value
            
        Raises:
            KeyError: If parameter does not exist
        """
        if name not in self.params:
            raise KeyError(f"Parameter {name} does not exist")
        return self.params[name].value
    
    def set(self, name, value):
        """
        Set parameter value
        
        Args:
            name (str): Parameter name
            value: New parameter value
            
        Raises:
            KeyError: If parameter does not exist
            ValueError: If parameter value type or range is incorrect
        """
        if name not in self.params:
            raise KeyError(f"Parameter {name} does not exist")
        self.params[name].value = value
        
    def get_param(self, name):
        """
        Get parameter object
        
        Args:
            name (str): Parameter name
            
        Returns:
            Parameter: Parameter object
            
        Raises:
            KeyError: If parameter does not exist
        """
        if name not in self.params:
            raise KeyError(f"Parameter {name} does not exist")
        return self.params[name]
    
    def get_all_params(self):
        """
        Get all parameters
        
        Returns:
            dict: Mapping from parameter names to parameter objects
        """
        return self.params
    
    def to_dict(self):
        """
        Convert all parameters to dictionary for JSON serialization
        
        Returns:
            dict: Parameter dictionary
        """
        return {name: param.to_dict() for name, param in self.params.items()}