"""Install exception handler for process crash."""
import os
import sys
import threading
import capnp
from selfdrive.version import version, dirty, origin, branch
import traceback
from datetime import datetime

from selfdrive.hardware import PC
from selfdrive.swaglog import cloudlog

if os.getenv("NOLOG") or os.getenv("NOCRASH") or PC:
  def capture_exception(*args, **kwargs):
    pass

  def bind_user(**kwargs):
    pass

  def bind_extra(**kwargs):
    pass

  def install():
    pass
else:
  from raven import Client
  from raven.transport.http import HTTPTransport
  from selfdrive.version import origin, branch, get_git_commit #, smiskol_remote
  from common.op_params import opParams
  import shutil

  CRASHES_DIR = '/data/community/crashes'
  if not os.path.exists(CRASHES_DIR):
    os.makedirs(CRASHES_DIR)

  error_tags = {'dirty': dirty, 'origin': origin, 'branch': branch, 'commit': get_git_commit(), 'username': opParams().get('username')}
  if error_tags['username'] is None or not isinstance(error_tags['username'], str):
    username = 'undefined'

  #if smiskol_remote:  # CHANGE TO YOUR remote and sentry key to receive errors if you fork this fork
    #sentry_uri = 'https://d544b36d2f81069aae01837c2fc53ef4@o4507756023906304.ingest.de.sentry.io/4507756026855504'
  #else:
    #sentry_uri = 'https://d544b36d2f81069aae01837c2fc53ef4@o4507756023906304.ingest.de.sentry.io/4507756026855504'  # stock
  client = Client(sentry_uri, install_sys_hook=False, transport=HTTPTransport, release=version, tags=error_tags)


  def save_exception(exc_text):
    log_file = '{}/{}'.format(CRASHES_DIR, datetime.now().strftime('%Y-%m-%d--%H:%M.log'))
    with open(log_file, 'w') as f:
      f.write(exc_text)
    shutil.copyfile(log_file, '{}/latest.log'.format(CRASHES_DIR))
    print('Logged current crash to {} and {}'.format(log_file, '{}/latest.log'.format(CRASHES_DIR)))

  def capture_exception(*args, **kwargs):
    save_exception(traceback.format_exc())
    exc_info = sys.exc_info()
    if not exc_info[0] is capnp.lib.capnp.KjException:
      client.captureException(*args, **kwargs)
    cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  def bind_user(**kwargs):
    client.user_context(kwargs)

  def bind_extra(**kwargs):
    client.extra_context(kwargs)

  def install():
    """
    Workaround for `sys.excepthook` thread bug from:
    http://bugs.python.org/issue1230540
    Call once from the main thread before creating any threads.
    Source: https://stackoverflow.com/a/31622038
    """
    # installs a sys.excepthook
    __excepthook__ = sys.excepthook

    def handle_exception(*exc_info):
      if exc_info[0] not in (KeyboardInterrupt, SystemExit):
        capture_exception()
      __excepthook__(*exc_info)
    sys.excepthook = handle_exception

    init_original = threading.Thread.__init__

    def init(self, *args, **kwargs):
      init_original(self, *args, **kwargs)
      run_original = self.run

      def run_with_except_hook(*args2, **kwargs2):
        try:
          run_original(*args2, **kwargs2)
        except Exception:
          sys.excepthook(*sys.exc_info())

      self.run = run_with_except_hook

    threading.Thread.__init__ = init
