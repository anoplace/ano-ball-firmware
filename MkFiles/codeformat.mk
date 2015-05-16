FIND=find
CLANG_FORMAT=clang-format

format:
	  $(FIND) . -name '*.[ch]' -exec $(CLANG_FORMAT) -i -style=Google {} \;
