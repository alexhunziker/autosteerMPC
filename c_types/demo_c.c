// compile with
// gcc -shared -Wl,-soname,demo_c -o demo_c.so -fPIC demo_c.c

int someFunction(int a, int b){
    return a + b;
}