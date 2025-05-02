#!/usr/bin/env python3

import argparse
import json
import os
import os.path
import shlex
import shutil
import socket
import subprocess
import multiprocessing
import logging
import re
import sys

#from pprint import pprint

DEBUG_OBJS_TO_COMPILE = []
C_COMPILER = None
CPP_COMPILER = None
CUR_SHA1 = None

def runCmd(cmd, change_to_path=None):
  """ Run the given cmd and return (retcode, stdout, stderr)

      If change_to_path != None, then change_to_path is passed to subprocess as
      cwd parameter; therefore we change to the given dir before running the cmd
  """
  if isinstance(cmd, str):
    cmd = shlex.split(cmd)
  if not isinstance(cmd, list):
    raise NotImplementedError

  if change_to_path is None:
    logging.debug('running "%s"', " ".join(cmd))
  else:
    logging.debug('changing to "%s" and running "%s"', change_to_path, " ".join(cmd))

  p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            cwd=change_to_path)
  stdout, stderr = p.communicate()
  stdout = stdout.decode('utf-8').rstrip()
  stderr = stderr.decode('utf-8').rstrip()
  retcode = p.returncode
#  logging.debug('retcode = %d. stderr = "%s", stdout = "%s"' % (retcode, stderr, stdout))
  return retcode, stdout, stderr


def getModificationTime(file_name):
    try:
        return os.path.getmtime(file_name)
    except OSError:
        return 0

def generateObject(obj, rules, objdir):
    fullpath_obj = "%s/%s" % (objdir, obj)
    fullpath_dir = os.path.dirname(fullpath_obj)
    if not os.path.exists(fullpath_dir):
        os.makedirs(fullpath_dir)
    for r in rules:
        if r.isApplicable(obj):
            # Returns tuple returned by runCmd
            return r.apply(obj, objdir)
    logging.error("No rule for object '%s'", obj)
    return False


def phase1Parser(allowed_to_recompile_parser=True):
    if not allowed_to_recompile_parser:
        rv = True
        if not os.path.exists('ext/mgpt/lexer.cc'):
            logging.error('Error! Not allowed to recompile lexer.cc but it does not exist\n')
            rv = False
        if not os.path.exists('ext/mgpt/parser.cc'):
            logging.error('Error! Not allowed to recompile parser.cc but it does not exist\n')
            rv = False
        return rv

    recompile_lexer = False
    if not os.path.exists('ext/mgpt/lexer.l'):
        recompile_lexer = True
    else:
        parser_mtime = getModificationTime('ext/mgpt/lexer.l')
        if parser_mtime > getModificationTime('ext/mgpt/lexer.cc'):
            recompile_lexer = True
    if recompile_lexer:
        retcode, stdout, stderr = runCmd(['lex', '-o', 'ext/mgpt/lexer.cc', 'ext/mgpt/lexer.l'])
        if retcode != 0:
            logging.error('Error recompiling lexer:\n%s', stderr)
            return False

    recompile_parser = False
    if not os.path.exists('ext/mgpt/parser.cc') or not os.path.exists('ext/mgpt/parser.h'):
        recompile_parser = True
    else:
        parser_mtime = getModificationTime('ext/mgpt/parser.y')
        if (parser_mtime > getModificationTime('ext/mgpt/parser.h')
                or parser_mtime > getModificationTime('ext/mgpt/parser.cc')):
            recompile_parser = True
    if recompile_parser:
        retcode, stdout, stderr = runCmd(['yacc', '-d', 'ext/mgpt/parser.y'])
        if retcode != 0:
            logging.error('Error recompiling parser:\n%s', stderr)
            return False
        if os.path.exists('ext/mgpt/parser.cc'):
            os.remove('ext/mgpt/parser.cc')
        os.rename('y.tab.c', 'parser.cc')
        shutil.move('parser.cc', 'ext/mgpt/')

        if os.path.exists('ext/mgpt/parser.h'):
            os.remove('ext/mgpt/parser.h')
        os.rename('y.tab.h', 'parser.h')
        shutil.move('parser.h', 'ext/mgpt/')
    return True


def phase2FindObjectsForLinkage(main_obj, deps):
    stack = [main_obj]
    # invariant: every time an item is pushed in the stack, it is also added in the
    # objs_needed_for_linking
    objs_needed_for_linking = [main_obj]
    while len(stack) > 0:
        i = stack.pop()
#         logging.debug('Processing %s' % i)
        if i not in deps:
#             logging.debug('%s has no dependency' % i)
            continue
        for prec in deps[i]:
            p = prec
            assert os.path.exists(p), "%s depends on %s but the latter doesn't exists" % (i, p)
            if prec[-2:] == '.h':
                p = prec[:-2]
                if os.path.exists(p + '.cc') or os.path.exists(p + '.c'):
                    p = p + '.o'
                else:
                    # header only
                    continue
            elif prec[-2:] == '.c':
                p = prec[:-2] + '.o'
            elif prec[-3:] == '.cc':
                p = prec[:-3] + '.o'
            if p not in objs_needed_for_linking:
#                 logging.debug("%s depends on %s (and it was added to stack)" % (i, p))
                stack.insert(0, p)
                objs_needed_for_linking.insert(0, p)
#             else:
#                 logging.debug("%s depends on %s (and it was already scheduled)" % (i, p))
    return objs_needed_for_linking


def compileIfNeeded(obj, deps, rules, objdir):
    obj_fullpath = "%s/%s" % (objdir, obj)
    need_to_compile = False
    if not os.path.exists(obj_fullpath):
        need_to_compile = True
    else:
        obj_mtime = getModificationTime(obj_fullpath)
        for prec in deps[obj]:
            prec_mtime = getModificationTime(prec)
            if obj_mtime < prec_mtime:
#                 print "%s is older than %s" % (obj, prec)
                need_to_compile = True

    # banner.o is always recompiled to make sure it has the correct date, git hash,
    # etc macros display on the banner
    if need_to_compile or obj == 'utils/banner.o':
        logging.info('Compiling %s', obj)
        rv = generateObject(obj, rules, objdir)
        return (True, obj_fullpath, rv)
    else:
        logging.debug('Reusing %s', obj)
        return (False, obj_fullpath, None)


def phase3Compiling(objs_needed_for_linking, deps, rules, objdir, n_threads=3):
    pool = multiprocessing.Pool(n_threads)
    ## FWT: compiled_all cannot be a basic variable because it will loose scope.
    ## a reference should be used instead
    status = {'compiled_all': True, 'compiled_some': False, 'stderr': []}


    def callbackCompiling(rv):
        was_compiled, obj_fullpath, gcc_rv = rv
#        logging.debug('callbackCompiling called with {}'.format(rv))
        if was_compiled == True:
            # This target was compiled
            status['compiled_some'] = True
            if gcc_rv == False or (isinstance(gcc_rv, tuple) and gcc_rv[0] != 0):
                status['compiled_all'] = False
                status['stderr'].append(gcc_rv[2])
                pool.close()
                pool.terminate()
            else:
                # Successfully compiled
                if len(gcc_rv[1]) > 0:
                    logging.info('Object "%s" compilied with success. Compiler stdout:\n%s',
                                 obj_fullpath, gcc_rv[1])
                if len(gcc_rv[2]) > 0:
                    logging.info('Object "%s" compilied with success. Compiler stderr:\n%s',
                                 obj_fullpath, gcc_rv[2])
                if not os.path.exists(obj_fullpath):
                    logging.error('Obj "%s" was compiled but could not be found... Python bug?',
                                  obj_fullpath)
                    status['compiled_all'] = False
                    status['stderr'].append('Not a gcc issue')
                    pool.close()
                    pool.terminate()


    global DEBUG_OBJS_TO_COMPILE
    DEBUG_OBJS_TO_COMPILE = objs_needed_for_linking

    for x in objs_needed_for_linking:
        pool.apply_async(compileIfNeeded, args=(x, deps, rules, objdir,),
                            callback=callbackCompiling)
    pool.close()
    pool.join()

    if not status['compiled_all']:
        logging.error("GCC failed to compile one or more files and returned:\n%s",
                        "\n".join(status['stderr']))
        return -1
    elif status['compiled_some']:
        return 1
    else:
        return 0



def phase4LinkTarget(objs_needed_for_linking, result_name, cpp_compiler, linker_flags, objdir):
    link_cmd = [cpp_compiler, '-o', result_name] \
                + ["%s/%s" % (objdir, obj) for obj in objs_needed_for_linking] \
                + linker_flags
    retcode, stdout, stderr = runCmd(link_cmd)
    if retcode != 0:
        logging.error("gcc returned '%d' when linking. Error message:\n%s", retcode, stderr)
        logging.debug("objects need to compile: [%s]", ", ".join(DEBUG_OBJS_TO_COMPILE))
        return False
    return True


def findAllSourceFiles(src_dir):
    all_files = []
    for root, dirnames, filenames in os.walk(src_dir):
        if '.git_ignore_me' in root:
            continue
        if root == '.':
            all_files += [f for f in filenames if f[-2:] == '.c' or f[-3:] == '.cc']
        else:
            assert root[0:2] == './'
            all_files += ["%s/%s" % (root[2:], f) for f in filenames if f[-2:] == '.c' or f[-3:] == '.cc']

#    print all_files
    return all_files


def objNameFromSrcName(src_file):
    base = src_file[:-1]  # Remove 'h' and 'c' from .h, .c, and .cc
    if base[-1] == 'c':  # was .cc
        base = base[:-1]
    return base + 'o'


def cleanPath(path_file):
    """ Clean paths such as 'a/b/../c/../d' to 'a/d'

        Useful because gcc -MM generates a lot of weird paths like that
    """
    path_parts = path_file.split('/')
    clean_path = []
    for i in path_parts:
        if i == '.':
            continue
        elif i == '..':
            clean_path.pop()
        else:
            clean_path.append(i)
#    print "cleanPath: %s -> %s" % (path_file, '/'.join(clean_path))
    return '/'.join(clean_path)


def generateDependency(src, include_flags):
    compiler = C_COMPILER
    flags = []
    if src[-3:] == '.cc':
        compiler = CPP_COMPILER
        flags.append('--std=c++20')
    cmd = [compiler, '-MM',] + flags + include_flags + [src]
    retcode, make_rule, stderr = runCmd(cmd)
    if retcode != 0:
        return (False, (retcode, make_rule, stderr))
    # logging.debug('Makefile rule for "%s": %s' % (src, make_rule))
    parts = make_rule.split()
    deps = set()
    # Ignoring the head of the rule, i.e., the .o file
    for i in parts[1:]:
        if i == '\\':
            continue
        deps.add(cleanPath(i))
    return (True, list(deps))


def generateDependencyWrapper(src, obj, include_flags):
    success, remaning_rv = generateDependency(src, include_flags)
    return (success, obj, remaning_rv)


def compileAndLinkWithAutoDependencies(main_obj, result_name, rules, deps_file, cpp_compiler,
                                        include_flags, linker_flags, objdir='.', n_threads=1,
                                        allowed_to_recompile_parser=True):
    pool = multiprocessing.Pool(n_threads)
    ## FWT: compiled_all cannot be a basic variable because it will loose scope.
    ## a reference should be used instead
    status = {'deps_changed': False, 'found_error': False, 'stderr': []}
    deps = {}

    def callbackAutoDep(rv):
        success, obj, dep_or_error_msg = rv
#        logging.debug('callbackAutoDep called with {}'.format(rv))
        if success == True:
            # This target was compiled
            if BOOST_HOME is not None:
                deps[obj] = [x for x in dep_or_error_msg if BOOST_HOME not in x]
            else:
                deps[obj] = dep_or_error_msg
            logging.debug('New dependency for "%s": [%s]', obj, ", ".join(dep_or_error_msg))
            status['deps_changed'] = True
        else:
            status['found_error'] = True
            status['stderr'].append(dep_or_error_msg[2])
            pool.close()
            pool.terminate()


    logging.info('Finding the dependencies for "%s"', main_obj)
    deps_mtime = 0
    if os.path.exists(deps_file):
        deps_mtime = getModificationTime(deps_file)
        with open(deps_file) as data_file:
            deps = json.load(data_file)

    all_srcs = findAllSourceFiles('.')
    for src in all_srcs:
        obj = objNameFromSrcName(src)
        if obj not in deps or getModificationTime(src) > deps_mtime:
            deps[obj] = None
            pool.apply_async(generateDependencyWrapper, args=(src, obj, include_flags,), callback=callbackAutoDep)

    pool.close()
    pool.join()

    if status['found_error']:
        logging.error('gcc found error while pre-processing for auto-dep:\n%s',
                      "\n".join(status['stderr']))
        logging.error('Giving up')
        return -1

    if status['deps_changed']:
        with open(deps_file, 'w') as data_file:
            json.dump(deps, data_file, separators=(',', ': '), indent=2)

    logging.info('Dependency generation done.')
    return compileAndLinkWithDependencies(main_obj, result_name, deps, rules, cpp_compiler,
                                            linker_flags, objdir, n_threads,
                                            allowed_to_recompile_parser)



# Return values:
#  -1: Error during compilation or linking
#   0: Nothing changed in the target, i.e., no compilation neither linking was
#      performed
#   1: Something changed and the target was compiled and linked with SUCCESS
def compileAndLinkWithDependencies(main_obj, result_name, deps, rules, cpp_compiler, linker_flags,
                                   objdir='.', n_threads=1, allowed_to_recompile_parser=True):
    # Phase 1: parser and lexer force for all binaries :(
    if not phase1Parser(allowed_to_recompile_parser):
        logging.error('Failed while generating the parser/lexer. Giving up')
        return -1

    # Phase 2: generating list of files that need to be compiled
    objs_needed_for_linking = phase2FindObjectsForLinkage(main_obj, deps)
    # TODO: HACK:
    if "ext/mgpt/problems.o" in objs_needed_for_linking:
        for forced_dep in ['ext/mgpt/parser.o', 'ext/mgpt/lexer.o']:
            if forced_dep not in objs_needed_for_linking:
                objs_needed_for_linking.append(forced_dep)

    # If there is at least one object to be compiled, then we recompile banner.o
    # because it has the macros showing time of compilation, closest git hash, etc.
    banner_obj = 'utils/banner.o'
    if len(objs_needed_for_linking) > 0 and banner_obj not in objs_needed_for_linking:
      objs_needed_for_linking.append('utils/banner.o')

    logging.info("Total objects needed for the target: %d", len(objs_needed_for_linking))
    logging.debug('Objects needed: %s', " ".join(sorted(objs_needed_for_linking)))


    # Phase 3: Compiling
    phase3_rv = phase3Compiling(objs_needed_for_linking, deps, rules, objdir, n_threads)
    if phase3_rv == -1:
        logging.error("Failed to compile one or more objects. Giving up")
        return -1
    elif phase3_rv == 0:
        ## Checking if any object is more recent than the target. If so, then we
        ## still need to link
        target_time = getModificationTime(result_name)
        need_to_be_linked = False
        for i in objs_needed_for_linking:
            if getModificationTime(objdir + "/" + i) > target_time:
                need_to_be_linked = True
                break
        if not need_to_be_linked:
            logging.info("Nothing was compiled for this target. Skipping linking phase")
            return 0
        else:
            logging.info("Linking is needed because another target changed objs of this target")

    # Phase 4: link together the necessary files
    logging.info('Linking binary for the giving target')
    if not phase4LinkTarget(objs_needed_for_linking, result_name, cpp_compiler, linker_flags, objdir):
        logging.error("Fail linking '%s'. Giving Up", result_name)
        return -1

    logging.info('Done linking binary')
    return 1



def performTests(tests_src, rules, deps_file, cpp_compiler, include_flags, linker_flags, objdir,
                 allowed_to_recompile_parser):
    i = 0
    for src in tests_src:
        i += 1
        logging.info("==== Test %d of %d: %s ====", i, len(tests_src), src)
        obj = objNameFromSrcName(src)
        binary = obj[:-2]
        compilation_status = compileAndLinkWithAutoDependencies(obj, binary, rules, deps_file,
                                                                cpp_compiler, include_flags,
                                                                linker_flags, objdir,
                                                                allowed_to_recompile_parser)
        if compilation_status == 1:
            logging.info("Running Test...")
            retcode, stdout, stderr = runCmd([os.path.basename(binary)], change_to_path=os.path.dirname(binary))
            if retcode != 0:
                with open('%s.out.git_ignore_me' % binary, 'w') as data_file:
                    data_file.write(stdout)
                with open('%s.err.git_ignore_me' % binary, 'w') as data_file:
                    data_file.write(stderr)
                logging.error("Test FAILED! Stopping. Logs at: %s.out.git_ignore_me and %s.err.git_ignore_me",
                                binary, binary)
                logging.error("Failed asserts:")
                for line in stdout.split('\n'):
                    if "FAILED:" in line:
                        logging.error("  " + line)
                # Deleting the test so it will be ran again (and compiled if needed)
                os.remove(objdir + '/' + obj)
                return -1
            else:
                logging.info("Test PASSED")
        elif compilation_status == 0:
            logging.info("This test didn't need to be compiled, so SKIPPING it.")
        elif compilation_status == -1:
            return -1
        else:
            logging.error("Unexpected return value '%s' from compileAndLinkWithAutoDependencies.",
                            str(compilation_status))
    return 1


def findAllTests():
    return [x for x in findAllSourceFiles('.') if x[0:6] == 'tests/' and x != 'tests/tests_utils.cc']


def clean(objdir):
    if os.path.exists(objdir):
        shutil.rmtree(objdir)
        os.makedirs(objdir)
    return 1


class Rule(object):
    def __init__(self):
        pass
    def isApplicable(self, obj):
        return None
    def apply(self, obj, objdir):
        return None

class GenericRule(Rule):
    def __init__(self, compiler, source_extension, flags):
        self.compiler = compiler
        self.source_extension = source_extension
        self.flags = flags
    def isApplicable(self, obj):
        root = obj[:-2]
        if obj[-2:] == '.o' and os.path.exists(root + self.source_extension):
            return True
        return False
    def apply(self, obj, objdir):
        source = "%s%s" % (obj[:-2], self.source_extension)
        cmd = [self.compiler] + self.flags + ['-c', source, '-o', "%s/%s" % (objdir, obj)]
        return runCmd(cmd)

class FileSpecificRule(Rule):
    def __init__(self, target, source, compiler, flags):
        self.target = target
        self.source = source
        self.compiler = compiler
        self.flags = flags
    def isApplicable(self, obj):
        return self.target == obj and os.path.exists(self.source)
    def apply(self, obj, objdir):
        cmd = [self.compiler] + self.flags + ['-c', self.source, '-o', "%s/%s" % (objdir, obj)]
        return runCmd(cmd)


class AddGitFlag(Rule):
    def __init__(self, rule):
        self.rule = rule
    def isApplicable(self, obj):
        return self.rule.isApplicable(obj)
    def apply(self, obj, objdir):
        git_hash = getClosestHashRef()
        self.rule.flags.append('-DGIT_HASH="%s"' % git_hash)
        return self.rule.apply(obj, objdir)


def getClosestHashRef():
    def runGitCmdWrapper(cmd):
        rv, stdout, stderr = runCmd(cmd)
        if rv != 0:
            logging.warning('%s returned "%d":\n%s', cmd, rv, stderr)
            return None
        return stdout

    if CUR_SHA1 is not None:
        logging.info('Using sha1 = "%s" given as parameter', CUR_SHA1)
        return CUR_SHA1

    logging.info('Finding closest git sha1 to current code')
    this_branch_name = runGitCmdWrapper("git rev-parse --abbrev-ref HEAD")
    if this_branch_name is None:
        return "UNKNOWN"

    rv, stdout, stderr = runCmd('git rev-parse "refs/wip/%s"' % this_branch_name)
    last_git_wip_name = stdout
    if rv == 0:
        timestamp_this_branch = runGitCmdWrapper("git show -s --format=%ct " + this_branch_name)
        if timestamp_this_branch is None:
            return "UNKNOWN"
        else:
            timestamp_this_branch = int(timestamp_this_branch)

        timestamp_wip = runGitCmdWrapper("git show -s --format=%ct " + last_git_wip_name)
        if timestamp_wip is None:
            return "UNKNOWN"
        else:
            timestamp_wip = int(timestamp_this_branch)

        if timestamp_this_branch < timestamp_wip:
            wip_sha1 = runGitCmdWrapper("git rev-parse %s" % last_git_wip_name)
            if wip_sha1 is None:
                return "UNKNOWN"
            else:
                return wip_sha1

    head_sha1 = runGitCmdWrapper("git rev-parse %s" % this_branch_name)
    if head_sha1 is None:
        return "UNKNOWN"
    else:
        return head_sha1



def main():
    parser = argparse.ArgumentParser(description='Build the code (Makefile substitute)')
    parser.add_argument('--keep_parser', '-k', action='store_true',
                        help='keep the current parser, i.e., don\'t call lex and yacc')
    parser.add_argument('--opt', action='store_true', help='turn on optimization flags')
    parser.add_argument('--ndebug', action='store_true', help='use flag -DNDEBUG to turn off all debug checks')
    parser.add_argument('--debug', action='store_true', help='turn on debug flags')
    parser.add_argument('--test', action='store_true', help='perform tests')
    parser.add_argument('--test-files', action='store_true', help='run the tests that uses the given files')
    parser.add_argument('--jobs', '-j', type=int, default=0, help='number of parallel jobs')
    parser.add_argument('--boost-home', type=str, dest='boost_home', default=None,
                        help='path to the home of boost, i.e., there should be lib/ and include/ there')
    parser.add_argument('-l', '--log', dest='logLevel',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
                        help="Set the logging level")
    parser.add_argument('-v', '--verbose', help="Print debugging statements",
                        action="store_const", dest="loglevel", const=logging.DEBUG,
                        default=logging.INFO)
    parser.add_argument('-c', '--c-compiler', help="C compiler",
                        type=str, default='gcc', dest='c_compiler')
    parser.add_argument('-p', '--cpp-compiler', help="C++ compiler",
                        type=str, default='g++', dest='cpp_compiler')
    parser.add_argument('-f', '--flag',
            help='Add the following flag to the C++ compiler. Exampe: -f="--pedantic -DMacroVal=3"',
            type=str, default=None, dest='cpp_flags')
    parser.add_argument('-b', '--branch-sha1', help="Branch SHA1 to be appended in the compiled file",
                        type=str, default=None, dest='cur_sha1')
    parser.add_argument('--lp_solver', choices=['gurobi', 'cplex'], default='gurobi',
            help='set which LP solver is compiled and used NOTE: not all code is available with cplex e.g. h_roc')
    parser.add_argument('--gprof', action='store_true', help='adds -pg flags to run gprof')
    parser.add_argument('--no_komihash', action='store_true', help='disables komihash')
    parser.add_argument('--no_phmap', action='store_true', help='disables phmap')

    parser.add_argument('target', type=str, help='target to be processed')
    args = parser.parse_args()

    if args.logLevel:
        log_format = '[%(levelname)s %(funcName)s::%(lineno)d %(asctime)s] %(message)s'
        logging.basicConfig(level=getattr(logging, args.logLevel), format=log_format, datefmt="%H:%M:%S")
    else:
        if args.test or args.test_files:
            # Not always on because the date confuses vim quickfix
            log_format = '[%(levelname)s %(asctime)s] %(message)s'
            logging.basicConfig(level=logging.INFO, format=log_format, datefmt="%H:%M:%S")
        else:
            log_format = '[%(levelname)s] %(message)s'
            logging.basicConfig(level=logging.INFO, format=log_format)


    n_threads = args.jobs

    hostname = socket.gethostname()

    global C_COMPILER
    C_COMPILER = args.c_compiler
    c_flags = ['-Wall', '-march=native']


    global CPP_COMPILER
    CPP_COMPILER = args.cpp_compiler

    global BOOST_HOME
    BOOST_HOME = args.boost_home

    global CUR_SHA1
    if args.cur_sha1 is not None:
        CUR_SHA1 = args.cur_sha1

    cpp_flags = ['-Wall', '-DATOM_STATES', '-DNO_STRICT', '-march=native', '-std=c++20']
    if args.cpp_flags is not None:
        cpp_cli_flags = args.cpp_flags.strip().split()
        logging.info("Adding the following C++ Flags: %s", str(cpp_cli_flags))
        cpp_flags += cpp_cli_flags

    if args.debug:
        debug_flags = ['-ggdb', '-DDIE_WITH_ASSERT']
        cpp_flags += debug_flags
        c_flags += debug_flags

    if args.opt:
        opt_flags = ['-ffloat-store', '-ffast-math', '-O3']
        cpp_flags += opt_flags
        c_flags += opt_flags

    if args.ndebug:
        cpp_flags += ['-DNDEBUG']
        c_flags += ['-DNDEBUG']


    include_flags = []
    linker_flags = []

    tmpdir = "/tmp/planner_tmp_" + str(hash(os.getcwd()))
    logging.info('Using "%s" as TMP directory', tmpdir)

    # Gurobi default values
    gurobi_home = '/usr/local/share/gurobi'
    gurobi_version = 81
    gurobi_cpp_library = '-lgurobi_c++'

    allowed_to_recompile_parser = True
    if args.keep_parser:
        allowed_to_recompile_parser = False

    logging.info('Hostname = "%s"', hostname)
    ### Host personalization
    if 'd12' in hostname or 'gpu' in hostname:
        logging.info("Host Personalization for d12/gpu")
        gurobi_cpp_library = '/usr/local/share/gurobi/lib/libgurobi_g++5.2.a'
        C_COMPILER = 'gcc-12'
        CPP_COMPILER = 'g++-12'
        cpp_flags.append('-Wno-deprecated-declarations')
        gurobi_version = 100
        if n_threads == 0:
            n_threads = 3
            logging.info(' - Using n_threads = {}'.format(n_threads))
    elif 'quntu' in hostname:
      logging.info("Host Personalization for quntu")
      gurobi_cpp_library      = '/opt/gurobi952/linux64/lib/libgurobi_g++5.2.a'
      gurobi_home             = '/opt/gurobi952/linux64'
      gurobi_version          = 95
      cplex_directory         = '/opt/ibm/ILOG/CPLEX_Studio201/cplex'
      cplex_concert_directory = '/opt/ibm/ILOG/CPLEX_Studio201/concert'
      C_COMPILER   = 'gcc-11'
      CPP_COMPILER = 'g++-11'
      if n_threads == 0:
          n_threads = 16
          logging.info(' - Using n_threads = {}'.format(n_threads))
    elif 'framework' in hostname:
      logging.info("Host Personalization for framework")
      gurobi_cpp_library      = '/opt/gurobi1103/linux64/lib/libgurobi_g++8.5.a'
      gurobi_home             = '/opt/gurobi1103/linux64'
      gurobi_version          = 110
      cplex_directory         = '/opt/ibm/ILOG/CPLEX_Studio201/cplex'
      cplex_concert_directory = '/opt/ibm/ILOG/CPLEX_Studio201/concert'
      C_COMPILER   = 'gcc-13'
      CPP_COMPILER = 'g++-13'
      if n_threads == 0:
          n_threads = 16
          logging.info(' - Using n_threads = {}'.format(n_threads))
    else:
        logging.info("No host personalization")

    rv, stdout, stderr = runCmd('%s --version' % CPP_COMPILER)
    m = re.search(" ([0-9])\.([0-9])\.([0-9]+)", stdout)
    if m is not None:
      v_major = int(m.group(1))
      v_middle = int(m.group(2))
      v_low = int(m.group(3))
      logging.info('C++ compiler: %s (%s)', CPP_COMPILER, m.group(0).strip())
      if 'g++' in CPP_COMPILER:
        if v_major > 4 and v_major < 8:
          logging.info('  This compiler should work (similar one has been used before)')
          if gurobi_cpp_library == '-lgurobi_c++':
            logging.warning("Are you sure that '%s' is the correct library? If compilation fails, point to the correct library (e.g., %s/lib/libgurobi_g++5.2.a) or recompile the Gurobi interface for c++", gurobi_cpp_library, gurobi_home)
        elif v_major == 4:
          if v_middle >= 9:
            logging.info('  This compiler should work (similar one has been used before)')
          else:
            logging.error("This version of gcc is KNOWN TO NOT WORK. Try 4.9 or later")
            sys.exit(-1)
        else:
          logging.info('  This compiler has not being used before. The recommendation is gcc 5')
    else:
      logging.warning('Failed to extract the compiler version from "%s"', stdout.split('\n')[0])

    if n_threads == 0:
        # Threads were not set and not defined in a profile. So using as
        # default 1
        n_threads = 1

    if BOOST_HOME is not None:
        include_flags.append('-I%s/include' % BOOST_HOME)
        linker_flags.append('-L%s/lib' % BOOST_HOME)

    # -------------------------------------------------------------------
    # -------------------- fmt -----------------------------------------
    # -------------------------------------------------------------------
    ### linker_flags.append('-lfmt')

    # -------------------------------------------------------------------
    # ------------------- gprof -----------------------------------------
    # -------------------------------------------------------------------
    if args.gprof:
        cpp_flags.append('-pg')
        linker_flags.append('-pg')

    # -------------------------------------------------------------------
    # ------------------- Faster hashmaps -------------------------------
    # -------------------------------------------------------------------
    if not args.no_komihash:
        cpp_flags.append('-DUSE_KOMIHASH')
    if not args.no_phmap:
        cpp_flags.append('-DUSE_PHMAP')

    # -------------------------------------------------------------------
    # ------------------ Gurobi -----------------------------------------
    # -------------------------------------------------------------------

    if args.lp_solver == 'gurobi':

        cpp_flags.append('-DUSE_GUROBI')
        c_flags.append('-DUSE_GUROBI')

        include_flags.append('-I%s/include/' % gurobi_home)
        linker_flags += ['-L%s/lib/' % gurobi_home,
                        gurobi_cpp_library,
                        '-lgurobi%d' % gurobi_version,
                        '-pthread',
                        '-lm']

    # -------------------------------------------------------------------
    # ------------------ CPLEX ------------------------------------------
    # -------------------------------------------------------------------

    elif args.lp_solver == 'cplex':

        # TODO(jsch): clean this up

        cpp_flags.append('-DUSE_CPLEX')
        c_flags.append('-DUSE_CPLEX')

        SYSTEM     = 'x86-64_linux'
        LIBFORMAT  = 'static_pic'

        CPLEXDIR      = cplex_directory
        CONCERTDIR    = cplex_concert_directory

        CPLEXLIBDIR   = '%s/lib/%s/%s' % (CPLEXDIR, SYSTEM, LIBFORMAT)
        CONCERTLIBDIR = '%s/lib/%s/%s' % (CONCERTDIR, SYSTEM, LIBFORMAT)

        CONCERTINCDIR = '%s/include' % CONCERTDIR
        CPLEXINCDIR   = '%s/include' % CPLEXDIR

        # CCFLAGS = '-I%s -I%s' % (CPLEXINCDIR, CONCERTINCDIR)
        include_flags += ['-I%s' % CPLEXINCDIR, '-I%s' % CONCERTINCDIR]


        # CCLNDIRS  = '-L%s -L%s' % (CPLEXLIBDIR, CONCERTLIBDIR)
        # CCLNFLAGS = '-lconcert -lilocplex -lcplex -lm -lpthread -ldl'
        linker_flags += ['-L%s' % CPLEXLIBDIR,
                            '-L%s' % CONCERTLIBDIR]
        linker_flags += ['-lconcert',
                            '-lilocplex',
                            '-lcplex',
                            '-lm',
                            '-lpthread',
                            '-ldl']

    else:
        logging.error('Unknown LP solver')
        exit(-1)

    # -------------------------------------------------------------------
    # -------------------------------------------------------------------
    # -------------------------------------------------------------------

    if not os.path.exists(tmpdir):
        os.makedirs(tmpdir)
    objdir = "%s/objs" % tmpdir
    deps_file = "%s/deps.json" % tmpdir

    logging.info('Using cpp_flags = %s', str(cpp_flags))
    logging.info('Using include_flags = %s', str(include_flags))
    logging.info('Using linker_flags = %s', str(linker_flags))
    used_cpp_flags = ' '.join(cpp_flags)

    rules = [
             AddGitFlag(FileSpecificRule('utils/banner.o', 'utils/banner.cc', CPP_COMPILER,
                              ['-DCFLAGS_USED="%s"' % used_cpp_flags,
                               '-DHOSTNAME="%s"' % hostname]
                              + cpp_flags + include_flags)),
             AddGitFlag(FileSpecificRule('solver_cssp.o', 'solver_cssp.cc', CPP_COMPILER,
                              ['-DCFLAGS_USED="%s"' % used_cpp_flags,
                               '-DHOSTNAME="%s"' % hostname]
                              + cpp_flags + include_flags)),
             AddGitFlag(FileSpecificRule('solver_ssp.o', 'solver_ssp.cc', CPP_COMPILER,
                              ['-DCFLAGS_USED="%s"' % used_cpp_flags,
                               '-DHOSTNAME="%s"' % hostname]
                              + cpp_flags + include_flags)),
             AddGitFlag(FileSpecificRule('eval_planner.o', 'eval_planner.cc', CPP_COMPILER,
                              ['-DCFLAGS_USED="%s"' % used_cpp_flags,
                               '-DHOSTNAME="%s"' % hostname]
                              + cpp_flags + include_flags)),
             FileSpecificRule('ext/mgpt/md4c.o', 'ext/mgpt/md4c.c', C_COMPILER, c_flags),
             # This rule is to prevent the warning on the code generated by FLEX
             FileSpecificRule('ext/mgpt/lexer.o', 'ext/mgpt/lexer.cc', CPP_COMPILER,
                              cpp_flags + ['-Wno-sign-compare']),
             GenericRule(CPP_COMPILER, '.cc', cpp_flags + include_flags),
            ]

    targets = {
       'solver_cssp': lambda: compileAndLinkWithAutoDependencies(
                         'solver_cssp.o', 'solver_cssp', rules, deps_file, CPP_COMPILER,
                          include_flags, linker_flags, objdir, n_threads,
                          allowed_to_recompile_parser),
       'solver_ssp': lambda: compileAndLinkWithAutoDependencies(
                         'solver_ssp.o', 'solver_ssp', rules, deps_file, CPP_COMPILER,
                          include_flags, linker_flags, objdir, n_threads,
                          allowed_to_recompile_parser),
       'eval_planner': lambda: compileAndLinkWithAutoDependencies(
                            'eval_planner.o', 'eval_planner', rules, deps_file,
                            CPP_COMPILER, include_flags, linker_flags, objdir, n_threads,
                            allowed_to_recompile_parser),
       'clean': lambda: clean(tmpdir)
    }

    # Aliases
    targets['ssp'] = targets['solver_ssp']
    targets['cssp'] = targets['solver_cssp']
    targets['all'] = (targets['ssp'], targets['cssp'])

    test_targets = {
        'all': findAllTests,
        'bw': ['tests/comparision_non_constrained_solution/comparision_non_constrained_solution_BW.cc'],
        'h-omc': lambda: [x for x in findAllTests() if 'tests/hacked_pr_sas_problem/' in x],
    }


    if args.test:
        logging.info('Processing TEST target "%s"', args.target)
        if args.target not in test_targets:
            logging.error('Unknown TEST target "%s"', args.target)
        else:
            if isinstance(test_targets[args.target], list) or isinstance(test_targets[args.target], tuple):
                tests_src = test_targets[args.target]
            else:
                tests_src = test_targets[args.target]()
            logging.info('Running %d tests', len(tests_src))
            rv = performTests(tests_src, rules, deps_file, CPP_COMPILER, include_flags, linker_flags,
                              objdir, allowed_to_recompile_parser)
            if rv < 0:
                logging.error('Tests FAILED!')
            else:
                logging.info('Tests were successful!')
    elif args.test_files:
        all_tests = findAllTests()
        logging.info("Searching for suitable tests within %d candidates", len(all_tests))
        chosen_tests = []
        for test in all_tests:
            (success, deps) = generateDependency(test, include_flags)
#             pprint(sorted(deps))
            if not success:
                logging.error("Error processing test '%s'", test)
            else:
                if args.target in deps:
                    logging.info("Scheduling test '%s'", test)
                    chosen_tests.append(test)
        if len(chosen_tests) == 0:
            logging.info("No relevant test found :(")
        else:
            rv = performTests(chosen_tests, rules, deps_file, CPP_COMPILER, include_flags, linker_flags,
                              objdir, allowed_to_recompile_parser)
            if rv < 0:
                logging.error('Tests FAILED!')
            else:
                logging.info('Tests were successful!')

    else:
        logging.info('Processing target "%s"', args.target)
        if args.target not in targets:
            logging.error('Unknown target "%s"', args.target)
        else:
            if isinstance(targets[args.target], list) or isinstance(targets[args.target], tuple):
                for sub_rule in targets[args.target]:
                    target_rule_rv = sub_rule()
                    if target_rule_rv < 0:
                        break
            else:
                target_rule_rv = targets[args.target]()

            if target_rule_rv < 0:
                logging.error('Failed at target "%s"', args.target)
            else:
                logging.info('Target "%s" built successfully', args.target)


if __name__ == "__main__":
    main()

