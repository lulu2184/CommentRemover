CommentRemover
==============
如果有regular和directory以外的文件类型如何处理？  
没有访问权限是否会导致出错？  
要新建目录的地方已经存在内容  
  
中间执行错误的输出  
怎么判断文件、目录是否为合法命名 是否需要判断  
  
  
判断是否有config  
判断输入目录是否存在  
  
Connect  
把那几个全局函数框起来  
factory和error的改名  
Q: 判断是否应该在folderprocessor的do里面？  
MatchExtension  
目录分隔符不要硬编码  
每个class的instance可以做成全局的，一开始就赋值好  
上级目录和当前目录 是否不要硬编码  
  
外面套一个类来判断是文件还是目录  
再写一个root的类  
