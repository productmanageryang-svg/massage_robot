package com.heacon.pad;

import android.app.Activity;
import android.graphics.Typeface;
import android.net.http.SslError;
import android.os.Bundle;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.View;
import android.view.WindowManager;
import android.webkit.PermissionRequest;
import android.webkit.SslErrorHandler;
import android.webkit.WebChromeClient;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.FrameLayout;
import android.widget.TextView;

import java.net.HttpURLConnection;
import java.net.URL;
import java.security.cert.X509Certificate;

import javax.net.ssl.HostnameVerifier;
import javax.net.ssl.HttpsURLConnection;
import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLSession;
import javax.net.ssl.TrustManager;
import javax.net.ssl.X509TrustManager;

public class MainActivity extends Activity {
    static final String[] PAGES = {
        "http://192.168.0.100:8765/",
        "https://192.168.0.100:8766/",
        "http://172.20.10.11:8765/",
        "https://172.20.10.11:8766/",
        "http://192.168.0.101:8765/",
        "https://192.168.0.101:8766/"
    };

    WebView web;
    TextView status;
    FrameLayout root;
    String pageUrl;
    boolean pageReady;
    final android.os.Handler clock = new android.os.Handler();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        hideSystemBars();
        trustAllSsl();

        status = new TextView(this);
        status.setText("正在连接电脑…");
        status.setTextColor(0xFF5C4636);
        status.setTextSize(TypedValue.COMPLEX_UNIT_SP, 22);
        status.setTypeface(Typeface.DEFAULT_BOLD);
        status.setGravity(Gravity.CENTER);
        status.setBackgroundColor(0xFFFFF9F4);

        web = new WebView(this);
        web.setBackgroundColor(0xFFFFF9F4);
        web.setVisibility(View.GONE);
        WebSettings s = web.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);
        s.setDatabaseEnabled(true);
        s.setMediaPlaybackRequiresUserGesture(false);
        s.setLoadWithOverviewMode(true);
        s.setUseWideViewPort(true);
        s.setAllowFileAccess(true);
        s.setAllowContentAccess(true);
        s.setMixedContentMode(WebSettings.MIXED_CONTENT_ALWAYS_ALLOW);
        s.setCacheMode(WebSettings.LOAD_NO_CACHE);
        s.setUserAgentString(s.getUserAgentString() + " HeaconPad");
        web.setWebViewClient(new WebViewClient() {
            @Override
            public void onReceivedSslError(WebView view, SslErrorHandler handler, SslError error) {
                handler.proceed();
            }

            @Override
            public void onPageFinished(WebView view, String url) {
                clock.removeCallbacksAndMessages(null);
                pageReady = true;
                web.setVisibility(View.VISIBLE);
                status.setVisibility(View.GONE);
            }

            @Override
            public void onReceivedError(WebView view, WebResourceRequest request, WebResourceError error) {
                if (request != null && request.isForMainFrame()) findDoorAgain();
            }

            @Override
            public void onReceivedError(WebView view, int errorCode, String description, String failingUrl) {
                if (failingUrl != null && pageUrl != null && failingUrl.startsWith(pageUrl)) findDoorAgain();
            }
        });
        web.setWebChromeClient(new WebChromeClient() {
            @Override
            public void onPermissionRequest(PermissionRequest request) {
                request.grant(request.getResources());
            }
        });

        root = new FrameLayout(this);
        root.setBackgroundColor(0xFFFFF9F4);
        root.addView(status, new FrameLayout.LayoutParams(
            FrameLayout.LayoutParams.MATCH_PARENT,
            FrameLayout.LayoutParams.MATCH_PARENT
        ));
        root.addView(web, new FrameLayout.LayoutParams(
            FrameLayout.LayoutParams.MATCH_PARENT,
            FrameLayout.LayoutParams.MATCH_PARENT
        ));
        setContentView(root);
        findDoorAgain();
    }

    void findDoorAgain() {
        pageReady = false;
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                status.setText("正在连接电脑…");
                status.setVisibility(View.VISIBLE);
                web.setVisibility(View.GONE);
            }
        });
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (!isFinishing()) {
                    for (int i = 0; i < PAGES.length; i++) {
                        final String url = PAGES[i];
                        if (reachable(url)) {
                            pageUrl = url;
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    status.setText("正在打开页面…");
                                    web.loadUrl(url);
                                    clock.removeCallbacksAndMessages(null);
                                    clock.postDelayed(new Runnable() {
                                        @Override
                                        public void run() {
                                            if (!pageReady) findDoorAgain();
                                        }
                                    }, 8000);
                                }
                            });
                            return;
                        }
                    }
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            status.setText("还没连上电脑\n请把平板连上机械臂同一个 Wi‑Fi");
                        }
                    });
                    try {
                        Thread.sleep(2500);
                    } catch (InterruptedException ignored) {
                        return;
                    }
                }
            }
        }).start();
    }

    static boolean reachable(String page) {
        HttpURLConnection conn = null;
        try {
            URL url = new URL(page);
            conn = (HttpURLConnection) url.openConnection();
            conn.setConnectTimeout(900);
            conn.setReadTimeout(900);
            conn.setInstanceFollowRedirects(true);
            conn.setRequestMethod("GET");
            int code = conn.getResponseCode();
            return code >= 200 && code < 500;
        } catch (Exception ignored) {
            return false;
        } finally {
            if (conn != null) conn.disconnect();
        }
    }

    static void trustAllSsl() {
        try {
            TrustManager[] trust = new TrustManager[] {
                new X509TrustManager() {
                    public void checkClientTrusted(X509Certificate[] c, String a) {}
                    public void checkServerTrusted(X509Certificate[] c, String a) {}
                    public X509Certificate[] getAcceptedIssuers() { return new X509Certificate[0]; }
                }
            };
            SSLContext ctx = SSLContext.getInstance("TLS");
            ctx.init(null, trust, new java.security.SecureRandom());
            HttpsURLConnection.setDefaultSSLSocketFactory(ctx.getSocketFactory());
            HttpsURLConnection.setDefaultHostnameVerifier(new HostnameVerifier() {
                public boolean verify(String hostname, SSLSession session) { return true; }
            });
        } catch (Exception ignored) {}
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) hideSystemBars();
    }

    @Override
    public void onBackPressed() {
        if (web != null && web.getVisibility() == View.VISIBLE && web.canGoBack()) web.goBack();
    }

    void hideSystemBars() {
        View decor = getWindow().getDecorView();
        decor.setSystemUiVisibility(
            View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                | View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
        );
    }
}
