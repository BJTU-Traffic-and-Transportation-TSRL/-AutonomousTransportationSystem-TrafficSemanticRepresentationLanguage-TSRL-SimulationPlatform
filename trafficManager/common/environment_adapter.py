"""
Environment Adapter for EnvCommunicator
适配Model类以提供EnvCommunicator所需的环境信息接口
"""

class EnvironmentAdapter:
    """
    适配器类，封装Model的功能以适配EnvCommunicator的需求
    """
    
    def __init__(self, model):
        """
        初始化环境适配器
        
        Args:
            model: Model实例，提供路网和环境信息
        """
        self.model = model
        self.junctions_cache = None
    
    def get_junctions_info(self):
        """
        获取所有交叉口信息
        
        Returns:
            dict: 交叉口ID到交叉口对象的映射
        """
        if self.junctions_cache is None and hasattr(self.model, 'nb'):
            self.junctions_cache = self.model.nb.junctions
        return self.junctions_cache or {}
    
    def get_junction_ids(self):
        """
        获取所有交叉口ID列表
        
        Returns:
            list: 交叉口ID列表
        """
        junctions = self.get_junctions_info()
        return list(junctions.keys()) if junctions else []
    
    def get_junction_by_id(self, junction_id):
        """
        根据ID获取特定交叉口
        
        Args:
            junction_id: 交叉口ID
            
        Returns:
            交叉口对象或None
        """
        junctions = self.get_junctions_info()
        return junctions.get(junction_id)
    
    def get_environment_status(self):
        """
        获取环境状态信息（适配EnvCommunicator接口）
        
        Returns:
            dict: 环境状态信息
        """
        return {
            "junctions": self.get_junction_ids(),
            "junction_count": len(self.get_junction_ids()),
            "scenario_name": getattr(self.model, 'Scenario_Name', 'unknown')
        }
    
    def get_weather_info(self):
        """
        获取天气信息（适配EnvCommunicator接口）
        
        Returns:
            dict: 天气信息，这里返回默认信息
        """
        return {
            "weather": "clear",
            "visibility": 1000,
            "temperature": 20
        }
    
    def get_road_condition(self):
        """
        获取道路条件信息（适配EnvCommunicator接口）
        
        Returns:
            dict: 道路条件信息
        """
        return {
            "road_condition": "dry",
            "surface_quality": "good"
        }