import os
import argparse
import importlib.util
import json
from pathlib import Path
import tempfile
import shutil
import subprocess
import unittest
from unittest.mock import patch


ENTRY = Path(__file__).resolve().parents[2] / 'deployment/runtime/application.py'
SPEC = importlib.util.spec_from_file_location('project_entry', ENTRY)
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)


def options(**values):
    return argparse.Namespace(**dict(no_app=False, stop=False, status=False, dry_run=False) | values)


class ProjectEntryTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.local = self.root / 'deployment/runtime/local'
        self.local.mkdir(parents=True)
        self.write('locations.json', {'schema_version': 1, 'host_role': 'windows_simulator',
                                     'wsl_distribution': 'Ubuntu-24.04'})
        self.write('launcher.json', {'schema_version': 1, 'wsl_project_root': '/home/user/My Project'})

    def write(self, name, value):
        (self.local / name).write_text(json.dumps(value), encoding='utf-8')

    def test_native_app_and_no_app(self):
        self.assertEqual(entry.command(options(), self.root, 'Linux'),
                         ['bash', str(self.root / 'start.sh')])
        value = entry.command(options(no_app=True, dry_run=True), self.root, 'Linux')
        self.assertEqual(value[-2:], ['--no-app', '--dry-run'])

    def test_windows_defaults_use_wsl_backend_then_windows_app(self):
        with patch.object(entry.shutil, 'which', return_value='pwsh.exe'), \
             patch.object(entry, 'require_matching_wsl_source') as verify:
            value = entry.command(options(), self.root, 'Windows')
        verify.assert_called_once_with(self.root, 'Ubuntu-24.04', '/home/user/My Project')
        self.assertIn('-SkipIsaac', value)
        self.assertEqual(value[-1], '/home/user/My Project')
        self.assertNotIn('-NoApp', value)
        self.assertEqual(value[:3], ['pwsh.exe', '-NoProfile', '-File'])

    def test_windows_no_app_and_dry_run(self):
        with patch.object(entry.shutil, 'which', return_value='pwsh.exe'), \
             patch.object(entry, 'require_matching_wsl_source'):
            value = entry.command(options(no_app=True, dry_run=True), self.root, 'Windows')
        self.assertEqual(value[-2:], ['-NoApp', '-DryRun'])

    def test_drift_refuses_every_start_but_not_stop_or_status(self):
        with patch.object(entry.shutil, 'which', return_value='pwsh.exe'), \
             patch.object(entry, 'require_matching_wsl_source', side_effect=ValueError('source differs')) as verify:
            for args in (options(), options(no_app=True), options(dry_run=True)):
                with self.assertRaisesRegex(ValueError, 'source differs'):
                    entry.command(args, self.root, 'Windows')
            verify.reset_mock()
            for args in (options(stop=True), options(status=True)):
                self.assertIn('./stop.sh', entry.command(args, self.root, 'Windows'))
            verify.assert_not_called()

    def test_matching_changed_added_removed_sources(self):
        before = {'one.py': 'a', 'removed.py': 'b'}
        with patch.object(entry, 'local_source_files', return_value=before), \
             patch.object(entry, 'wsl_source_state', return_value={'exists': True, 'files': dict(before)}):
            entry.require_matching_wsl_source(self.root, 'Ubuntu', '/home/user/source')
        with patch.object(entry, 'local_source_files', return_value=before), \
             patch.object(entry, 'wsl_source_state', return_value={'exists': True, 'files': {'one.py':'x','extra.py':'y'}}):
            with self.assertRaisesRegex(ValueError, '3 files'):
                entry.require_matching_wsl_source(self.root, 'Ubuntu', '/home/user/source')

    def test_source_probe_uses_current_rules_and_is_read_only(self):
        rules = (ENTRY.with_name('release_source_manifest.py')).read_text()
        source = self.root / 'ros2_ws/src/demo/main.cpp'
        source.parent.mkdir(parents=True);source.write_text('original')
        config = self.local / 'locations.json'
        before = config.read_bytes()
        program = f'MODULE = {rules!r}\n' + entry.SOURCE_PROBE
        result = subprocess.run([__import__('sys').executable, '-', str(self.root)],
                                input=program, capture_output=True, text=True, check=True)
        state = json.loads(result.stdout)
        self.assertTrue(state['exists'])
        self.assertIn('ros2_ws/src/demo/main.cpp', state['files'])
        self.assertNotIn('deployment/runtime/local/locations.json', state['files'])
        self.assertEqual(before, config.read_bytes())

    def test_old_python_refused_before_child_process(self):
        with patch.object(entry.sys, 'version_info', (3, 10)), patch.object(entry.subprocess, 'run') as run:
            self.assertEqual(entry.main([]), 2)
            run.assert_not_called()

    def test_stop_and_status_only_delegate_existing_stop_lifecycle(self):
        for system in ('Linux', 'Windows'):
            for args, suffix in ((options(stop=True), 'stop.sh'),
                                 (options(status=True), '--status'),
                                 (options(stop=True, dry_run=True), '--dry-run')):
                value = entry.command(args, self.root, system)
                self.assertIn('stop.sh', ' '.join(value))
                self.assertNotIn('start.sh', ' '.join(value))
                self.assertTrue(value[-1].endswith(suffix))
                if system == 'Windows':
                    self.assertEqual(value[value.index('--exec') + 1], 'bash')
                    self.assertNotIn('--', value)

    def test_operator_stop_confirmation_is_forwarded_only_for_stop(self):
        args=options(stop=True);args.simulation_stopped=True
        for system in ('Linux','Windows'):
            self.assertIn('--simulation-stopped',entry.command(args,self.root,system))
        args=options();args.simulation_stopped=True
        with self.assertRaisesRegex(ValueError,'only valid with Stop'):
            entry.command(args,self.root,'Windows')

    def test_windows_stop_closes_app_only_after_successful_stop(self):
        def run(arguments, **_):
            calls.append(arguments)
            return subprocess.CompletedProcess(arguments, stop_code if len(calls) == 1 else 0)
        for system, argv, stop_code, closes in (('Windows', ['--stop'], 0, True),
                                                ('Windows', ['--stop', '--simulation-stopped'], 0, True),
                                                ('Windows', ['--stop'], 2, False),
                                                ('Windows', ['--status'], 0, False),
                                                ('Windows', ['--stop', '--dry-run'], 0, False),
                                                ('Linux', ['--stop'], 0, False)):
            calls = []
            with patch.object(entry, 'command', return_value=['wsl-stop']), \
                 patch.object(entry.platform, 'system', return_value=system), \
                 patch.object(entry.shutil, 'which', return_value='pwsh.exe'), \
                 patch.object(entry.subprocess, 'run', side_effect=run):
                self.assertEqual(entry.main(argv), stop_code, (system, argv))
            self.assertEqual(calls[0], ['wsl-stop'])
            self.assertEqual(len(calls), 2 if closes else 1, (system, argv))
            if closes:
                self.assertEqual(calls[1][-2:], [str(ENTRY.parents[2] / 'deployment/windows/start_app.ps1'), '-CloseApp'])

    def test_windows_special_path_is_a_single_exec_argument(self):
        path = "/home/user/project with spaces;$(touch forbidden)'quote"
        self.write('launcher.json', {'schema_version': 1, 'wsl_project_root': path})
        value = entry.command(options(status=True), self.root, 'Windows')
        self.assertEqual(value[value.index('--cd') + 1], path)
        self.assertIn('--exec', value)
        self.assertNotIn('--', value)

    @unittest.skipUnless(shutil.which('pwsh') or (shutil.which('pwsh.exe') if os.name == 'nt' else None), 'PowerShell 7 unavailable')
    def test_powershell_browser_owner_selection_and_handoff(self):
        source = str(ENTRY.parents[2] / 'deployment/windows/start_app.ps1').replace("'", "''")
        script = r"""
$ErrorActionPreference = 'Stop'
$tokens = $null; $errors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile('SOURCE', [ref] $tokens, [ref] $errors)
if ($errors.Count) { throw ($errors | Out-String) }
$helpers = $ast.FindAll({ param($node)
    $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
    $node.Name -in @('Get-DedicatedBrowserProcess', 'Wait-DedicatedBrowserProcess', 'Close-DedicatedBrowserApp')
}, $true)
foreach ($helper in $helpers) { . ([scriptblock]::Create($helper.Extent.Text)) }
$profile = Join-Path ([IO.Path]::GetTempPath()) 'App Profile'
$browser = Join-Path ([IO.Path]::GetTempPath()) 'chrome.exe'
$script:rows = @(
    [pscustomobject]@{ProcessId=41; CommandLine="chrome.exe --user-data-dir=`"$profile`" --app=http://127.0.0.1:8088/"},
    [pscustomobject]@{ProcessId=42; CommandLine="chrome.exe --user-data-dir=`"$profile`" --type=renderer"},
    [pscustomobject]@{ProcessId=43; CommandLine="chrome.exe --user-data-dir=`"$profile-other`""}
)
function Get-CimInstance { param($ClassName, $Filter, $ErrorAction)
    if ($ClassName -ne 'Win32_Process' -or $Filter -ne "Name = 'chrome.exe'") { throw 'Unscoped browser query' }
    return $script:rows
}
function Get-Process { param($Id, $ErrorAction) return [pscustomobject]@{Id=[int]$Id} }
$owner = Get-DedicatedBrowserProcess -Browser $browser -Profile $profile
if ($owner.Id -ne 41) { throw 'Did not select exact main dedicated-profile owner' }
$script:rows = @($script:rows[1], $script:rows[2])
if ($null -ne (Get-DedicatedBrowserProcess -Browser $browser -Profile $profile)) { throw 'Attached unrelated browser/renderer' }
$script:waits = [Collections.Generic.List[int]]::new()
$a = [pscustomobject]@{Id=100;ExitCode=0}
$b = [pscustomobject]@{Id=101;ExitCode=0}
foreach ($process in @($a,$b)) {
    $process | Add-Member -MemberType ScriptMethod -Name WaitForExit -Value { $script:waits.Add($this.Id) }
}
$script:replacement = $b
function Get-DedicatedBrowserProcess { param($Browser, $Profile)
    $value = $script:replacement; $script:replacement = $null; return $value
}
$code = Wait-DedicatedBrowserProcess -Process $a -Browser $browser -Profile $profile
if ($code -ne 0 -or ($script:waits -join ',') -ne '100,101') { throw 'Did not follow actual profile-owner handoff' }
$a.ExitCode=7
$code = Wait-DedicatedBrowserProcess -Process $a -Browser $browser -Profile $profile
if ($code -ne 7) { throw 'Browser failure became successful App close' }
# Stop closes the owner gracefully first and force-stops only that process on timeout.
$script:events = [Collections.Generic.List[string]]::new()
function New-App { param($Id, $Exits)
    $app = [pscustomobject]@{Id=$Id; Exits=$Exits}
    $app | Add-Member -MemberType ScriptMethod -Name CloseMainWindow -Value { $script:events.Add("close $($this.Id)"); $true }
    $app | Add-Member -MemberType ScriptMethod -Name WaitForExit -Value { param($ms) $script:events.Add("wait $($this.Id)"); $result = $this.Exits[0]; $this.Exits = @($this.Exits | Select-Object -Skip 1); $result }
    return $app
}
$script:owners = @{'msedge.exe' = (New-App 200 @($true)); 'chrome.exe' = (New-App 201 @($false, $true))}
function Get-DedicatedBrowserProcess { param($Browser, $Profile)
    if ($Profile -ne 'P') { throw 'Wrong profile' }
    return $script:owners[$Browser]
}
function Stop-Process { param($Id, [switch] $Force, $ErrorAction) $script:events.Add("kill $Id") }
Close-DedicatedBrowserApp -Profile 'P' -TimeoutSeconds 0 | Out-Null
if (($script:events -join ',') -ne 'close 200,wait 200,close 201,wait 201,kill 201,wait 201') { throw "Unexpected close sequence: $($script:events -join ',')" }
$script:owners = @{}
$script:events.Clear()
Close-DedicatedBrowserApp -Profile 'P' | Out-Null
if ($script:events.Count) { throw 'Signalled a browser without the dedicated profile' }
'Browser ownership mocks passed'
""".replace('SOURCE', source)
        result = subprocess.run([shutil.which('pwsh') or (shutil.which('pwsh.exe') if os.name == 'nt' else None),
                                 '-NoProfile', '-Command', script],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Browser ownership mocks passed', result.stdout)

    def test_missing_setup_is_actionable(self):
        (self.local / 'launcher.json').unlink()
        with self.assertRaisesRegex(ValueError, 'Setup'):
            entry.command(options(), self.root, 'Windows')

    def test_invalid_or_shared_windows_checkout_rejected(self):
        for value in ('relative', '/home/user/../project', '/mnt/c/project', '/mnt/D/project',
                      '/home/user/project\nmore'):
            self.write('launcher.json', {'schema_version': 1, 'wsl_project_root': value})
            with self.assertRaises(ValueError, msg=value):
                entry.command(options(), self.root, 'Windows')

    def test_wrong_host_role_and_distribution_rejected(self):
        for role, distro in (('wsl_application', 'Ubuntu'), ('windows_simulator', '-bad'),
                             ('windows_simulator', 'Ubuntu\nother')):
            self.write('locations.json', {'schema_version': 1, 'host_role': role,
                                         'wsl_distribution': distro})
            with self.assertRaises(ValueError):
                entry.command(options(), self.root, 'Windows')

    def test_missing_pwsh_and_unsupported_os(self):
        with patch.object(entry.shutil, 'which', return_value=None):
            with self.assertRaisesRegex(ValueError, 'PowerShell'):
                entry.command(options(), self.root, 'Windows')
        with self.assertRaisesRegex(ValueError, 'Supported application hosts'):
            entry.command(options(), self.root, 'Darwin')

    def test_child_failure_propagates_without_retry(self):
        with patch.object(entry, 'command', return_value=['backend']), \
                patch.object(entry.subprocess, 'run') as child:
            child.return_value.returncode = 7
            self.assertEqual(entry.main(['--no-app']), 7)
            child.assert_called_once()

    def test_command_error_does_not_spawn_a_process(self):
        with patch.object(entry, 'command', side_effect=ValueError('Setup required')), \
                patch.object(entry.subprocess, 'run') as child:
            self.assertEqual(entry.main([]), 2)
            child.assert_not_called()


if __name__ == '__main__':
    unittest.main()
