import xml.etree.ElementTree as ET

# 加载SUMO输出的XML文件
tree = ET.parse('tripinfo.xml')
root = tree.getroot()

# 遍历所有tripinfo元素
for tripinfo in root.findall('tripinfo'):
    # 获取当前id属性值
    current_id = tripinfo.get('id')
    # 检查id是否以'bg.'开头
    if current_id and current_id.startswith('bg.'):
        # 移除'bg.'前缀，只保留数字部分
        new_id = current_id[3:]
        tripinfo.set('id', new_id)

# 保存修改后的XML文件
tree.write('tripinfo_modified.xml', encoding='utf-8', xml_declaration=True)