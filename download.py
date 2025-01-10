from baidupcsapi import PCS
 
# 替换为你的用户名和密码
pcs = PCS('1335654481ren', '2468922ren')
print(pcs.quota())  # 输出你的存储配额信息
print(pcs.list_files('/'))  # 输出根目录下的文件列表