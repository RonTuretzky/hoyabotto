// On GitHub Pages, emoji-config.js supplies the public HTTPS visitor endpoint.
// On the local service, an empty base keeps all calls on the same origin.
window.emojiFetch = async (path, options = {}) => {
  const base = (window.ROBOT_EMOJI_API || '').replace(/\/$/, '');
  try {
    const response = await fetch(base + path, {
      ...options, cache: 'no-store', credentials: 'omit', signal: AbortSignal.timeout(8000)
    });
    const body = await response.json();
    if (!response.ok) throw new Error(body.error || 'ショーは現在ご利用いただけません。少ししてからもう一度どうぞ。');
    return body;
  } catch (error) {
    if (error instanceof TypeError || error.name === 'TimeoutError' || error.name === 'SyntaxError') {
      throw new Error(options.method === 'POST'
        ? '接続が切れました。すでに並んでいるかもしれません。もう一度送る前に画面を確認してください。'
        : 'ロボットのショーは現在オフラインです。少ししてからもう一度どうぞ。');
    }
    throw error;
  }
};
