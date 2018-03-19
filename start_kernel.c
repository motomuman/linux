#include <stdio.h>
#include <lkl_host.h>
//compile  clang -I/home/motomu/linux/tools/lkl/include -o start_kernel ./start_kernel.c /home/motomu/linux/tools/lkl/liblkl.a -lrt -lpthread

int main(){
	int ret;
	ret = lkl_start_kernel(&lkl_host_ops);
	if (ret) {
		fprintf(stderr, "can't start kernel: %s\n", lkl_strerror(ret));
		return 0;
	} else {
		fprintf(stderr, "started kernel\n");
	}
}
