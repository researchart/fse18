#!/usr/bin/env perl
# Author: Jamie Davis <davisjam@vt.edu>
# Description: Run complete regex structural analysis
#
# Usable as a workScript for distributed-map.pl
#
# Dependencies:
#   - ECOSYSTEM_REGEXP_PROJECT_ROOT must be defined
#
# Options:
#   - ANALYSES  Which analyses to run? X,Y,...
#               Choose from: any-qoa any-qod count-star-height is-linear-engine-compatible
#               Default is all of them.
#   - MINIMUM_REPETITION_UPPER_LIMIT X
#               How large of an upper limit is "too large" for safety? {0,10} is OK but {0,40} is not.
#               Default is 25 based on safe-regex.
#   - COUNT_QUESTION_MARKS           0|1
#               Whether to count question marks as repetition. You should not use this. 
#               Default is 0. Only for compatibility with safe-regex.
#
# Emits to stdout the input object with an additional object 'structuralAnalysis'.
# Members (depending on the analyses you select):
#   anyQOA                    0|1
#   anyQOD                    0|1
#   starHeight                0 <= X
#   isLinearEngineCompatible  0|1
# Each may have the value 'UNKNOWN' if the regex was not susceptible to the analysis.

use strict;
use warnings;

use JSON::PP;

# Check dependencies.
if (not defined $ENV{ECOSYSTEM_REGEXP_PROJECT_ROOT}) {
  die "Error, ECOSYSTEM_REGEXP_PROJECT_ROOT must be defined\n";
}

# Command-line args
if (scalar(@ARGV) ne 1) {
  die "Usage: $0 regex-file.json\n";
}

my $regexFile = $ARGV[0];
my $cont = &readFile("file"=>$regexFile);
my $regexObj = decode_json($cont);
&log("Pattern: /$regexObj->{pattern}/");

# Globals
my %ANALYSES = &getAnalyses();
my @ANALYSES_NAMES = keys %ANALYSES;

## Env-var args.

# ANALYSES
my @analyses_names = @ANALYSES_NAMES;
if ($ENV{ANALYSES}) {
  @analyses_names = split(",", $ENV{ANALYSES});
}
# Unique.
my %names;
map { $names{$_} = 1 } @analyses_names;
@analyses_names = keys %names;

# Valid names?
for my $a (@analyses_names) {
  if (not &contains(\@ANALYSES_NAMES, $a)) {
    die "Error, unsupported analysis <$a>. Choose from <@ANALYSES_NAMES>\n";
  }
}
&log("ANALYSES <@analyses_names>");

# MINIMUM_REPETITION_UPPER_LIMIT
my $MINIMUM_REPETITION_UPPER_LIMIT = 25;
if ($ENV{MINIMUM_REPETITION_UPPER_LIMIT}) {
  $MINIMUM_REPETITION_UPPER_LIMIT = int($ENV{MINIMUM_REPETITION_UPPER_LIMIT});
}
&log("MINIMUM_REPETITION_UPPER_LIMIT $MINIMUM_REPETITION_UPPER_LIMIT");

# COUNT_QUESTION_MARKS
my $COUNT_QUESTION_MARKS = 0;
if ($ENV{COUNT_QUESTION_MARKS}) {
  $COUNT_QUESTION_MARKS = int($ENV{COUNT_QUESTION_MARKS});
}
&log("COUNT_QUESTION_MARKS $COUNT_QUESTION_MARKS");

# Extend regex obj with options.
$regexObj->{options} = { "minimumRepetitionUpperLimit" => int($MINIMUM_REPETITION_UPPER_LIMIT),
                         "countQuestionMarks"          => int($COUNT_QUESTION_MARKS),
                       };

# Prep the query file.
my $queryFile = "/tmp/run-structural-analysis-pid$$-queryFile-regexWithOptions.json";
unlink $queryFile;
&writeToFile("file"=>$queryFile, "contents"=>encode_json($regexObj));

# Query each analysis.
my $overallResultObj = {};
for my $analysis_name (@analyses_names) {
  my ($rc, $out) = &cmd_OutputToFile("$ANALYSES{$analysis_name}->{wrapper} $queryFile");
  if ($rc) {
    $overallResultObj->{$ANALYSES{$analysis_name}->{resultName}} = "FAILED";
  }
  else {
    my $result = decode_json($out);
    $overallResultObj->{$ANALYSES{$analysis_name}->{resultName}} = $result->{$ANALYSES{$analysis_name}->{resultName}};
  }
}

# Print output
$regexObj->{structuralAnalysis} = $overallResultObj;

print STDOUT encode_json($regexObj) . "\n";
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

# input: ($cmd) should not redirect output
# output: ($rc, $out) captures stdout via a file, sends stderr to /dev/null
sub cmd_OutputToFile {
  my ($cmd) = @_;

  my $tmpFile = "/tmp/run-structural-analysis-pid$$.out";
  unlink $tmpFile;

  my ($rc, $out) = &cmd("$cmd > $tmpFile 2>/dev/null");
  $out = &readFile("file"=>$tmpFile);
  unlink $tmpFile;

  return ($rc, $out);
}

sub log {
  my ($msg) = @_;
  print STDERR "$msg\n";
}

# input: (\@list, $e)
# output: true if $e is in @list, else false
sub contains {
  my ($list, $e) = @_;
  for my $elt (@$list) {
    if ($elt eq $e) {
      return 1;
    }
  }

  return 0;
}

# input: ()
# output: (%name2details)
#   name: shorthand
#   details: keys: wrapper resultName
#     wrapper: absolute path to the analysis wrapper
#     resultName: member of the result object containing the result
sub getAnalyses {
  my $wrapperPrefix = "$ENV{ECOSYSTEM_REGEXP_PROJECT_ROOT}/structural-analysis/";

  my %name2wrapper;
  $name2wrapper{"any-qoa"} = { "resultName" => "anyQOA" };
  $name2wrapper{"any-qod"} = { "resultName" => "anyQOD" };
  $name2wrapper{"count-star-height"} = { "resultName" => "starHeight" };
  $name2wrapper{"is-linear-engine-compatible"} = { "resultName" => "isLinearEngineCompatible" };

  for my $name (keys %name2wrapper) {
    $name2wrapper{$name}->{wrapper} = "$wrapperPrefix/$name.js";
  }
  
  # Confirm validity
  for my $name (keys %name2wrapper) {
    if (not -x $name2wrapper{$name}->{wrapper}) {
      die "Error, cannot execute wrapper for $name: $name2wrapper{$name}->{wrapper}\n";
    }
  }
 
  return %name2wrapper;
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

# input: %args: keys: file
# output: $contents
sub readFile {
  my %args = @_;

	open(my $FH, '<', $args{file}) or die "Error, could not read $args{file}: $!";
	my $contents = do { local $/; <$FH> }; # localizing $? wipes the line separator char, so <> gets it all at once.
	close $FH;

  return $contents;
}
