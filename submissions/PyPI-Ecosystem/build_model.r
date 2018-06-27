
library(htmlTable)
library(OIsurv)
library(survival)
library(car)
library(survminer)
library(ggplot2)

# Only do this once
# sd <- read.csv("survival_data.pypi_2008_2017-12_6.csv")
# sd$age_start = c(0, unlist(lapply(2:nrow(sd),
#                                   function(x){ ifelse(sd[(x-1),"name"] == sd[x,"name"],
#                                                       sd[(x-1),"age"],
#                                                       0) })))
# write.table(sd, file="survival_data.csv", sep=";", row.names = FALSE)

# sd.noskip <- read.csv("no_skip_last_year.csv")
# sd.noskip$age_start = c(0, unlist(lapply(2:nrow(sd.noskip),
#                                   function(x){ ifelse(sd.noskip[(x-1),"name"] == sd.noskip[x,"name"],
#                                                       sd.noskip[(x-1),"age"],
#                                                       0) })))
# write.table(sd.noskip, file="no_skip_last_year.repeated.csv", sep=";", row.names = FALSE)


sd <- read.csv("survival_data.csv", sep=";")

# View(sd[1:100,])

sd$license_type = as.factor(sd$license_code)
table(sd$license_type)

names(sd)
nrow(sd)
length(unique(sd$name))



sd$fy = sd$age <= 12
sd$fhy = sd$age <= 6
sd$has_university = sd$university>=0.1
sd$has_commercial = sd$commercial>0.1
sd$has_backporting = sd$backporting==1
sd$upstreams_sq = sd$upstreams ^ 2
sd$downstreams_sq = sd$downstreams ^ 2
sd$commits_sq = sd$commits ^ 2

library(sqldf)
age_stats = sqldf("select name, min(month) as 'start_month',
                max(age) as 'death_age', max(dead) as 'dead'
               from sd group by name")
View(head(age_stats))
table(age_stats$start_month=="2016-12")
table(age_stats$start_month=="2016-11")
# boxplot(dead ~ death_age, age_stats[age_stats$death_age<18,])

pdf(file="pypi-per-month.pdf", width=6, height=3.5)
par(mar=c(4, 4, 1, 0))
plot(table(age_stats[age_stats$start_month!="2016-12",]$start_month),
     # type = "l",
     ylab="Number of new PyPI packages", xlab="Time", xaxt="n")
axis(1, at=c(1,13,25,37,49,61,73,85,97,109),
     labels=c("2008-01","2009-01","2010-01","2011-01","2012-01",
             "2013-01","2014-01","2015-01","2016-01","2017-01"))
dev.off()

hist(age_stats[!(age_stats$name %in% age_stats[age_stats$start_month<"2012-01",]$name),]$death_age)
# ag = age_stats[!(age_stats$name %in% age_stats[age_stats$start_month<"2012-01",]$name),]$death_age
# length(ag)
ag = age_stats[!(age_stats$name %in% age_stats[age_stats$start_month<"2012-01",]$name) &
                 age_stats$dead==1,]$death_age
length(ag)
plot(table(ag[ag>1]))

# Putting death_age = 0 and death_age = 1 together because of the way
# the data was collected: months are calendar months (if project starts
# on Jan 31st and dies on Feb 1st, death_age = 1)
dead_prematurely = age_stats[age_stats$dead &
                               (age_stats$death_age==0 |
                                  age_stats$death_age==1),]$name
length(dead_prematurely)

# dead_prematurely6 = age_stats[age_stats$dead &
#                                (age_stats$death_age<6),]$name
# length(dead_prematurely6)



# Split the data to model premies separately
length(unique(subset(sd, !(name %in% age_stats[age_stats$start_month<"2012-01",]$name))$name))

nrow(sd)
sd.prem = subset(sd, !(name %in% unique(age_stats[age_stats$start_month<"2012-01",]$name)))
nrow(sd.prem)
length(unique(sd.prem$name))
sd.prem = subset(sd.prem, age<6)
                   # (name %in% dead_prematurely | age<=1))
                  # (name %in% dead_prematurely6 | age<6))
nrow(sd.prem)
length(unique(sd.prem$name))
table(sd.prem$dead)

sd.prem.sample = subset(sd.prem) #, age_start!=age)
nrow(sd.prem.sample)
# sd.prem.sample = rbind(sd.prem[sd.prem$dead==0,],
#                        sd.prem[sd.prem$dead==1,][sample(nrow(sd.prem[sd.prem$dead==1,]),
#                                                         nrow(sd.prem[sd.prem$dead==0,])),])

nrow(sd.prem.sample)
table(sd.prem.sample$dead)

# t = table(sd.prem.f$name)
# head(t[t>1])

sd.prem.sample.f = subset(sd.prem.sample,
                          commits<=exp(6) &
                            contributors<=exp(3.5) &
                            q90<=1 &
                            issues<=exp(5) &
                            submitters<=exp(5) &
                            downstreams<40 &
                            t_downstreams<=exp(8) &
                            dc_katz<0.001 &
                            non_dev_issues<5 &
                            cc_degree<50 &
                            upstreams<30)

m.prem = glm(dead ~ log(commits+1) +
                    # log(commits_sq + 1) +
                   log(contributors+1) +
                   # log(q90+1) +
                   # log(issues+1) +
                   log(non_dev_issues+1) +
                   # log(submitters+1) +
                   # log(non_dev_submitters+1) +
                   log(cc_degree+1) +
                   # log(downstreams+1) +
               downstreams +
                   # log(t_downstreams+1) +
                   # log(upstreams+1) +
               upstreams +
                    # log(d_upstreams+1) +
               d_upstreams +
                   # log(t_upstreams+1),
                   scale(log(dc_katz+1)) +
                   has_university +
                    has_commercial +
                   # log(commercial+1) +
                   has_backporting +
                    license_type +
                   org,
             family="binomial",
             data=sd.prem.sample.f)


vif(m.prem)
summary(m.prem)
Anova(m.prem, type=2)
library(pscl)
pR2(m.prem)
# plot(m.prem)


source("helpers.r")
library(texreg)
library(xtable)

file="tex_model_glm.csv"
modelNames=c("Early-stage abandoners")
caption="Early-stage abandoners model; response: abandoned=TRUE"

mList = list(m1=m.prem)
makeTexRegCox(mList, file, modelNames, caption, digits=2)

print_Anova_glm(m.prem, "anova_model_glm_1.csv")


# print_anova(rddFit, "~/R/anova_model_fresh_rdd_1.csv")






# 2-month+ survivors

sd.f = subset(sd, !(name %in% dead_prematurely) &
                !(name %in% age_stats[age_stats$start_month<"2012-01",]$name) &
                  age_start!=age & age>=6)
nrow(sd.f)
table(sd.f$dead)


# sd.0 = subset(sd, age_start!=age)
# sfit0 = survfit(Surv(sd.0$age_start, sd.0$age, sd.0$dead) ~ 1)

sd.f = subset(sd, age_start!=age & !(name %in% unique(age_stats[age_stats$start_month<"2012-01",]$name)))
s = Surv(sd.f$age_start, sd.f$age, sd.f$dead)
sfit = survfit(s ~ 1)
# plot(sfit, xlab="months", ylab="proportion not dead")

library(ggplot2)
ggsurvplot(
  sfit,
  # sfit0,
  data = sd.f,
  # data = sd.0,
  size = 1,                 # change line size
  palette = c("#2E9FDF"),
    #c("#E7B800", "#2E9FDF"),# custom color palettes
  conf.int = TRUE,          # Add confidence interval
  # pval = TRUE,              # Add p-value
  # risk.table = TRUE,        # Add risk table
  # risk.table.col = "strata",# Risk table color by groups
  # legend.labs =
    # c("Male", "Female"),    # Change legend labels
  # risk.table.height = 0.25, # Useful to change when you have multiple groups
  legend = "none",
  xlab = "Time in months",   # customize X axis label.
  break.time.by = 12,     # break X axis in time intervals by 500.
  ggtheme = theme_light()
  # ggtheme = theme_bw()      # Change ggplot2 theme
)
ggsave("base-survival.pdf", width = 6, height = 3.5)


# s = Surv(sd$age, sd$dead)
# sfit = survfit(s ~ 1)
# plot(sfit)

sd.f$commits_time = log(sd.f$commits+1) * sd.f$age_start
sd.f$downstreams_time = log(sd.f$downstreams+1) * log(sd.f$age_start+1)
sd.f$upstreams_time = log(sd.f$upstreams+1) * log(sd.f$age_start+1)
sd.f$q90_time = log(sd.f$q90+1) * log(sd.f$age_start)
sd.f$has_downstreams = sd.f$downstreams > 0

y = subset(sd.f,
           commits<=exp(6) &
             contributors<=exp(3.5) &
             q90<=exp(2.5) &
             issues<=exp(5) &
             submitters<=exp(4) &
             upstreams<=exp(4) &
             downstreams<=500 &
             t_downstreams<=exp(8) &
             dc_katz<0.002 &
             cc_degree<30)
hist(log(y$commits+1))
hist(log(y$contributors+1))
hist(log(y$q90+1))
hist(log(y$issues+1))
hist(log(y$submitters+1))
hist(log(y$upstreams+1))
hist(log(y$downstreams+1))
boxplot(log(downstreams+1) ~ age, y)
boxplot(log(upstreams+1) ~ age, y)
boxplot(log(upstreams_time+1) ~ age, y)
boxplot(log(q90+1) ~ age, y)


sm_repeating <- coxph(Surv(age_start, age, dead) ~
# sm_repeating <- coxph(Surv(age, dead) ~
                        # cluster(name) +
                        log(commits+1) +
                        log(contributors+1) +
                        # log(q90+1) +
                        # q90_time +
                        log(non_dev_issues+1) +
                        # log(issues+1) +
                        # log(non_dev_issues+1) + #* log(contributors+1) +
                        # log(submitters+1) +
                        # log(non_dev_submitters+1) +
                        log(cc_degree+1) +
                        # log(downstreams+1) +
                        downstreams +
                        # downstreams_time +
                        upstreams +
                        d_upstreams +
                        # log(upstreams_sq+1) +
                        # upstreams_time +
                        # log(t_downstreams+1) +
                        # log(t_upstreams+1),
                        scale(log(dc_katz+1)) +
                        has_university +
                        has_commercial +
                        has_backporting +
                        license_type +
                        org,
                      data=y)
                      # data=sd.prem.sample.f)

summary(sm_repeating)


hist(log(sd.prem.sample.f$commits+1))
hist(log(sd.prem.sample.f$contributors+1))
hist(log(sd.prem.sample.f$q90+1))
hist(log(sd.prem.sample.f$issues+1))
hist(log(sd.prem.sample.f$submitters+1))
hist(log(sd.prem.sample.f$upstreams+1))
hist(log(sd.prem.sample.f$downstreams+1))
boxplot(log(downstreams+1) ~ age, sd.prem.sample.f)
boxplot(downstreams ~ age, sd.prem.sample.f)
boxplot(log(upstreams+1) ~ age, sd.prem.sample.f)
boxplot(log(q90+1) ~ age, sd.prem.sample.f)
boxplot(dead ~ age, sd.prem.sample.f)


vif(sm_repeating)

# library("survminer")
# options("scipen"=100, "digits"=4)
cox.zph(sm_repeating) %>%
  extract2("table") %>%
  txtRound(digits = 2) %>%
  knitr::kable(align = "r")
test.ph = cox.zph(sm_repeating)
ggcoxzph(test.ph)

Anova(sm_repeating, type=2)

round(exp(sm_repeating$coefficients), 2)


sm_repeating_inter <- coxph(Surv(age_start, age, dead) ~
                        # sm_repeating <- coxph(Surv(age, dead) ~
                        # cluster(name) +
                        log(commits+1) * log(non_dev_issues+1) +
                        # log(contributors+1) +
                        # log(q90+1) +
                        # q90_time +
                        # log(non_dev_issues+1) +
                        # log(issues+1) +
                          log(contributors+1) * log(non_dev_issues+1) +
                        # log(submitters+1) +
                        # log(non_dev_submitters+1) +
                        log(cc_degree+1) +
                        # log(downstreams+1) +
                        downstreams +
                        # downstreams_time +
                          # log(upstreams+1)+
                        upstreams +
                          # log(upstreams_sq+1)+
                        upstreams_sq +
                        d_upstreams +
                          # log(d_upstreams+1)+
                        # upstreams_time +
                        # log(t_downstreams+1) +
                        # log(t_upstreams+1),
                        scale(log(dc_katz+1)) +
                        has_university +
                        has_commercial +
                        has_backporting +
                        license_type +
                        org,
                      data=subset(y)) #, downstreams<=10 & upstreams<=20))

summary(sm_repeating_inter)
vif(sm_repeating_inter)


source("helpers.r")
library(texreg)
library(xtable)

file="tex_model_cox.csv"
modelNames=c("Abandoners")
caption="All abandoners except early-stage model"

mList = list(m1=sm_repeating)
makeTexRegCox(mList, file, modelNames, caption, digits=2)

print_Anova_glm(sm_repeating, "anova_model_cox_1.csv")



file="tex_model_all.csv"
modelNames=c("Early-stage abandoners","Non-early-stage abandoners","Non-early-stage abandoners")
caption="All abandoners except early-stage model"

mList = list(m1=m.prem, m2=sm_repeating, m3=sm_repeating_inter)
makeTexRegCox(mList, file, modelNames, caption, digits=2)

print_Anova_glm(m.prem, "anova_model_all_1.csv")
print_Anova_glm(sm_repeating, "anova_model_all_2.csv")
print_Anova_glm(sm_repeating_inter, "anova_model_all_3.csv")

# print_anova(rddFit, "~/R/anova_model_fresh_rdd_1.csv")





################ BELOW BE PUMPKINS ################

# sd.f = subset(sd, age_start!=age)
nrow(sd)
sd.f = droplevels(
      subset(sd, age_start!=age &
                !(name %in% age_stats[age_stats$start_month<"2012-01",]$name) &
                commits<=exp(6) &
                contributors<=exp(3.5) &
                q90<=exp(3) &
                issues<=exp(5) &
                submitters<=exp(5) &
                upstreams<=exp(4) &
                downstreams<=exp(6.5) &
                t_downstreams<=exp(8) &
                dc_katz<=0.01)
)
nrow(sd.f)


# Projects being born per month
# plot(table(sd[sd$age==0,]$month))
# Projects dying per month
# plot(table(sd[sd$dead==1,]$month))


plot(table(sd.f$month))



sd_last <- sd[sd$last_observation == 1, ]
sd_new <- sd_last[sample(nrow(sd_last), 2000), ]

cox.fit.last <- coxph(Surv(age, dead) ~
           + q90
           , data=sd_new)
summary(cox.fit.last)

cox.zph.last <- survival::cox.zph(cox.fit.last)
plot(cox.zph.last)

sd_last <- read.csv("survival_data4R_last.csv")
# sd_last$university <- as.integer(sd_last$uni > 0.2)
sd_last$commercial <-  as.integer(sd_last$comm > 0.2)
sd_last$trivial <- as.integer(sd_last$max_contrib <= 1)

sd_full <- read.csv("survival_data4R.csv")
# sd_full$university <- as.integer(sd_full$uni > 0.2)
sd_full$commercial <-  as.integer(sd_full$comm > 0.2)
sd_full$trivial <- as.integer(sd_full$max_contrib <= 1)

?survival::Surv

# baseline hazard
surv_obj = survfit(Surv(age, dead) ~ 1, data=sdl)
plot(surv_obj, xlab="months", ylab="proportion not dead")
title("Baseline survival function")

sd_last_split = survSplit(
    Surv(age, dead) ~ .,
    data=sd_last,
    cut=c(1),
    episode = 'tgroup'
)
head(sd_last_split)

cox.fit.last <- coxph(Surv(age, dead) ~
           + log(contributors1 + 1)
           + q90
           + log(non_dev_submitters + 1)
           + log(closed_issues + 1)
           + log(connectivity1000 + 1)
           + a_downstreams
           + upstreams
           + d_upstreams0
           + org
           + university
           + cc_closeness_1
           , data=sd_last)
summary(cox.fit.last)

cox.zph.last <- cox.zph(cox.fit.last)
cox.zph.last

cox.zph.full <- cox.zph(cox.fit.full)
cox.zph.full

cox.fit.full <- coxph(Surv(age, dead) ~
           + log(contributors1 + 1)
           + q90
           + log(non_dev_submitters + 1)
           + log(closed_issues + 1)
           + log(connectivity1000 + 1)
           + a_downstreams
           + upstreams
           + d_upstreams0
           + org
           + university
           + cc_closeness_1
           , data=sd_full)
summary(cox.fit.full)

cox.zph.full <- cox.zph(cox.fit.full)
cox.zph.full

ggcoxzph(cox.zph.full)

?sample

# sample dataset to reduce memory consumption
ggcoxdiagnostics(cox.fit.full)

surv_obj = survfit(Surv(age, dead) ~ !commercial, data=sdl)
plot(surv_obj, xlab="months", ylab="proportion not dead", col=c("blue", "red"))
legend("topright", c("commercial", "not"), col=c("blue", "red"), lty=1)
title("Commercial vs not")

?legend

surv_obj = survival::survfit(survival::Surv(age, dead) ~ !uni, data=sdl)
plot(surv_obj, xlab="months", ylab="proportion not dead", col=c("blue", "red"))
legend("topright", c("university", "not"), col=c("blue", "red"), lty=1)
title("University vs not")

surv_obj = survival::survfit(survival::Surv(age, dead) ~ !trivial, data=sdl)
plot(surv_obj, xlab="months", ylab="proportion not dead", col=c("blue", "red"))
legend("topright", c("trivial", "not"), col=c("blue", "red"), lty=1)
title("Trivial vs not")

# sd is just the same data without first year duplicates for compatibility
sd = sdl[(sdl$age > 11) | sdl$dead,]
head(sd)

s <- coxph(Surv(age, dead) ~
           # fy  # S INSIGNIFICANT
           + log(contributors1 + 1)
           # + fy*log(contributors1 + 1)  # S INSIGNIFICANT
           #
           + q90  #+ gini # + commits # + q50 + q70
           + fy*q90
           + log(non_dev_submitters + 1)
           # + fy*log(non_dev_submitters + 1)  # S INSIGNIFICANT
           #
           + log(closed_issues + 1)
           # + fy*log(closed_issues + 1)  # P INSIGNIFICANT
           #
           + log(connectivity1000 + 1)
           # + fy*log(connectivity1000 + 1)  # S INSIGNIFICANT
           #
           + a_downstreams # + a_downstreams + ac_downstreams + c_downstreams
           + fy*a_downstreams
           #
           + upstreams
           # + fy*upstreams  # S INSIGNIFICANT
           #
           + d_upstreams0
           # + fy*d_upstreams0 # NA
           #
           + org
           + fy*org
           #
           + university
           # + fy*university   # P INSIGNIFICANT
           # + commercial
#            + dc_katz
#            + fy*dc_katz
           #
           + cc_closeness_1
           # + fy*cc_closeness_1  # P INSIGNIFICANT
           , data=sdl)
summary(s)

vif(s)

anova(s)

plot(survfit(s), xlab="months", ylab="proportion not dead")

# TODO:
# get number of commits in general for active developers
# also number of projects

s_nov <- coxph(Surv(age, dead) ~
           + log(contributors1 + 1)
           + q90  #+ gini # + commits # + q50 + q70
           + log(non_dev_submitters + 1)
#            + non_dev_issues + issues  # useless
           + log(closed_issues + 1)
           # connectivity in the last month/year
           + log(connectivity1000 + 1)
           + a_downstreams # + a_downstreams + ac_downstreams + c_downstreams
           # upstreams / a_upstreams:
           + upstreams
           # binarize dead upstreams
           + d_upstreams0
           + org
           + dc_katz
           + cc_closeness_1
           , data=sd)
summary(s_nov)

vif(s_nov)

anova(s_nov)

# Repeating observations
# Index([u'age', u'date', u'project', u'q50', u'gini', u'contributors',
#        u'upstreams', u'commits', u'submitters', u'c_upstreams',
#        u'c_downstreams', u'dead', u'ac_downstreams', u'connectivity',
#        u'non_dev_issues', u'a_downstreams', u'closed_issues', u'ac_upstreams',
#        u'a_upstreams', u'downstreams', u'issues', u'q70'],
#       dtype='object')

# TO_CHECK (1|project_id) - random effect term
sm_repeating <- coxph(Surv(age, dead) ~ cluster(project)
            + log(contributors + 1) + q70 + gini
           + submitters
           + closed_issues
           + log(connectivity + 1)
           + a_downstreams
           + a_upstreams
           , data=sd)
summary(sm_repeating)

vif(sm_repeating)

plot(survfit(s_nov), xlab="months", ylab="proportion not dead")

library(car)

vif(s)
