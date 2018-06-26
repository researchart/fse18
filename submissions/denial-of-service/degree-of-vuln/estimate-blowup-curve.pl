#!/usr/bin/env perl
# Author: Jamie Davis <davisjam@vt.edu>
# Description: Estimate the blow-up rate of a vulnerable regex
# Query should define:
#   - language   Which language to test? X,Y,...
#   - pattern
#   - evilInput  As defined by some detector.
#   - pumpStart  We test X*a^0, X*a^1, X*a^2, X*a^3, ... , for a = 1.1
#   - maxPumps   Cap the exponent.
#   - timeLimit  In seconds.

use strict;
use warnings;

use JSON::PP; # I/O
use Time::HiRes qw( gettimeofday tv_interval );
use Carp;
use POSIX qw(ceil);

my $DEBUG = 0;
if ($ENV{REGEX_DEBUG}) {
  $DEBUG = 1;
}

my $GEOMETRIC_FACTOR = 1.1;

# Check dependencies.
if (not defined $ENV{ECOSYSTEM_REGEXP_PROJECT_ROOT}) {
  die "Error, ECOSYSTEM_REGEXP_PROJECT_ROOT must be defined\n";
}

my $validateVuln = "$ENV{VULN_REGEX_DETECTOR_ROOT}/src/validate/validate-vuln.pl";
my $fitCurve = "$ENV{ECOSYSTEM_REGEXP_PROJECT_ROOT}/degree-of-vuln/fit-curve.py";

for my $script ($validateVuln, $fitCurve) {
  if (not -x $script) {
    die "Error, script is not executable: $script\n";
  }
}

# Check args.
if (scalar(@ARGV) != 1) {
  die "Usage: $0 query.json\n";
}

my $queryFile = $ARGV[0];
if (not -f $queryFile) {
  die "Error, no such file: $queryFile\n";
}

my $cont = &readFile("file"=>$queryFile);
my $query = decode_json($cont);

my @requiredKeys = ("language", "pattern", "evilInput", "pumpStart", "maxPumps", "timeLimit");
for my $key (@requiredKeys) {
  if (not defined $query->{$key}) {
    die "Error, you must define key $key\n";
  }
}

&log("Testing blowup curve of pattern /$query->{pattern}/");

my $tmpFile = "/tmp/test-blowup-curve-$$.json";
my $tmpFile_curveFit = "/tmp/test-blowup-curve-$$-curveFit.csv"; 
unlink $tmpFile, $tmpFile_curveFit;
my ($rc, $out);

###############

my @resultsForDiffPumps;

my $nPumps = $query->{pumpStart};
for (my $pumpIx = 1; $pumpIx < $query->{maxPumps}; $pumpIx++) {
  my $t0 = [gettimeofday];

  my $validateVulnQuery = {
    "language" => $query->{language},
    "pattern" => $query->{pattern},
    "evilInput" => $query->{evilInput},
    "nPumps" => $nPumps,
    "timeLimit" => $query->{timeLimit}
  };
  &writeToFile("file"=>$tmpFile, "contents"=>encode_json($validateVulnQuery));

  my $ONE_GB_IN_BYTES = 1*1024*1024*1024;
  my $memoryLimit = 1 * $ONE_GB_IN_BYTES;
  my $memoryLimitCmds = "ulimit -m $memoryLimit; ulimit -v $memoryLimit";
  my ($rc, $out) = &cmd("$memoryLimitCmds; timeout $query->{timeLimit}s $validateVuln $tmpFile 2>/dev/null");
  my $elapsed = tv_interval($t0);

  my $result = { "nPumps"     => $nPumps,
                 "elapsedSec" => $elapsed
               };
  # Timeout?
  if ($rc eq 124) {
    $result->{timedOut} = 1;
    $result->{invalidPattern} = 0;
  }
  # Failed? (presumably a crash on unsupported pattern)
  elsif ($rc) {
    $result->{invalidPattern} = 1;
  }
  # Things worked!
  else {
    $result->{timedOut} = 0;
    $result->{invalidPattern} = 0;
  }

  push @resultsForDiffPumps, $result;

  if ($result->{timedOut}) {
    &log("Timed out, no point in trying again");
    last;
  }

  $nPumps = ceil($nPumps * $GEOMETRIC_FACTOR);
}

&log("resultsForDiffPumps:\n" . encode_json(\@resultsForDiffPumps));

# We have a curve, find a fit.
&log("Fitting curve");
my $header = "nPumps,matchTime";
my @csvData = map { "$_->{nPumps},$_->{elapsedSec}" } @resultsForDiffPumps;
&writeToFile("file"=>$tmpFile_curveFit, "contents"=> join("\n", $header, @csvData));
my $result = decode_json(&chkcmd("$fitCurve $tmpFile_curveFit 2>/dev/null"));

$query->{curve} = $result;
print STDOUT encode_json($query) . "\n";

unlink $tmpFile unless $DEBUG;
unlink $tmpFile_curveFit unless $DEBUG;
exit 0;

#####################

# input: ($cmd)
# output: ($rc, $out)
sub cmd {
  my ($cmd) = @_;
  &log("CMD: $cmd");
  my $out = `$cmd`;
  return ($? >> 8, $out);
}

sub chkcmd {
  my ($cmd) = @_;
  my ($rc, $out) = &cmd($cmd);
  if ($rc) {
    die "Error, cmd failed. rc $rc out\n$out\n";
  }

  return $out;
}

sub log {
  my ($msg) = @_;
  print STDERR "$msg\n";
}


# input: ()
# output: (@detectors) fields: name wrapper
#   name: shorthand
#   wrapper: absolute path to the detector wrapper
sub getDetectors {
  my $wrapperPrefix = "$ENV{ECOSYSTEM_REGEXP_PROJECT_ROOT}/analyze-regexps/redos/detectors";

  # field: name
  my @detectors = ( {"name" => "rathnayake-rxxr2"},
                    {"name" => "weideman-RegexStaticAnalysis"},
                    {"name" => "wuestholz-RegexCheck"},
                    {"name" => "substack-safe-regex"}
                  );
  # field: wrapper
  for my $d (@detectors) {
    $d->{wrapper} = "$wrapperPrefix/query-$d->{name}.pl";
  }

  # Confirm validity
  for my $d (@detectors) {
    if (not can_run($d->{wrapper})) {
      die "Error, cannot run wrapper for $d->{name}: $d->{wrapper}\n";
    }
  }
 
  return @detectors;
}

# input: (\@list, $e)
# output: true if $e is in @list, else false
sub listContains {
  my ($list, $e) = @_;
  for my $elt (@$list) {
    if ($elt eq $e) {
      return 1;
    }
  }

  return 0;
}

# input: %args: keys: file
# output: $contents
sub readFile {
  my %args = @_;

	open(my $FH, '<', $args{file}) or confess "Error, could not read $args{file}: $!";
	my $contents = do { local $/; <$FH> }; # localizing $? wipes the line separator char, so <> gets it all at once.
	close $FH;

  return $contents;
}

# input: %args: keys: file contents
# output: $file
sub writeToFile {
  my %args = @_;

	open(my $fh, '>', $args{file});
	print $fh $args{contents};
	close $fh;

  return $args{file};
}
