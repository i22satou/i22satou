public class Main {
    public static void main(String[] args) {
        int n = 10; // 表示するフィボナッチ数列の長さ
        long[] fib = new long[n];

        // フィボナッチ数列の初期値を設定
        fib[0] = 0;
        if (n > 1) {
            fib[1] = 1;
        }

        // フィボナッチ数列を計算
        for (int i = 2; i < n; i++) {
            fib[i] = fib[i - 1] + fib[i - 2];
        }

        // フィボナッチ数列を表示
        for (int i = 0; i < n; i++) {
            System.out.println("Fibonacci(" + i + ") = " + fib[i]);
        }
    }
}