// On GitHub Pages, emoji-config.js supplies the public HTTPS visitor endpoint.
// On the local service, an empty base keeps all calls on the same origin.
window.emojiFetch = async (path, options = {}) => {
  const base = (window.ROBOT_EMOJI_API || '').replace(/\/$/, '');
  try {
    const response = await fetch(base + path, {
      ...options, cache: 'no-store', credentials: 'omit', signal: AbortSignal.timeout(8000)
    });
    const body = await response.json();
    if (!response.ok) throw new Error(body.error || 'The show is unavailable. Please try again later.');
    return body;
  } catch (error) {
    if (error instanceof TypeError || error.name === 'TimeoutError' || error.name === 'SyntaxError') {
      throw new Error(options.method === 'POST'
        ? 'Connection interrupted. Your request may already be queued; check the screen before sending again.'
        : 'The robot show is offline. Please try again in a moment.');
    }
    throw error;
  }
};
